import sys
import os
import json
import numpy as np
from pathlib import Path
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QLineEdit, QPushButton, QListWidget, QGroupBox,
    QStatusBar, QFileDialog, QRadioButton, QButtonGroup, QProgressBar,
    QInputDialog, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime

# Import the GPS IK bindings
# User will select the .pyd file on first run
gk = None

def compute_robot_tool_to_ee_tip(roll_deg, pitch_deg):
    """
    Compute robotTool_X_eeTip transform using the chain from MotionSessionServiceImpl.
    Returns an Affine3d object.
    """
    # Convert to radians
    roll_rad = np.deg2rad(roll_deg)
    pitch_rad = np.deg2rad(pitch_deg)
    ten_deg_rad = -np.pi / 18.0
    
    # Helper to create 4x4 homogeneous matrix
    def make_transform(rotation=None, translation=None):
        T = np.eye(4)
        if rotation is not None:
            T[:3, :3] = rotation
        if translation is not None:
            T[:3, 3] = translation
        return T
    
    # Rotation matrices
    def rot_x(angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    
    def rot_y(angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    
    # 1. tool_X_intermediate
    tool_X_intermediate = make_transform(
        rotation=np.eye(3),
        translation=[14.241998, -4.96991, 41.963651]
    )
    
    # 2. intermediate_X_A
    intermediate_X_A = make_transform(
        rotation=rot_y(ten_deg_rad),
        translation=[61.564791, 4.893199, -80.820654]
    )
    
    # 3. A_X_B (roll rotation around X)
    A_X_B = make_transform(rotation=rot_x(roll_rad))
    
    # 4. B_X_intermediate
    B_X_intermediate = make_transform(
        rotation=np.eye(3),
        translation=[3.18368127222, 0, 10.8452958304]
    )
    
    # 5. intermediate_X_ee (pitch rotation around Y + rotated offset)
    rot_pitch = rot_y(-pitch_rad)
    offset_homogeneous = np.array([16.687501, 0, 7.290719, 1])
    rotated_offset = rot_pitch @ offset_homogeneous[:3]
    intermediate_X_ee = make_transform(
        rotation=rot_pitch,
        translation=rotated_offset
    )
    
    # Chain multiplication
    result = tool_X_intermediate @ intermediate_X_A @ A_X_B @ B_X_intermediate @ intermediate_X_ee
    
    # Convert numpy matrix to Affine3d
    # Extract rotation (as unit vectors) and translation
    x_axis = gk.UnitVector3d.unitX()  # We'll set these from the matrix
    y_axis = gk.UnitVector3d.unitY()
    z_axis = gk.UnitVector3d.unitZ()
    origin = gk.Point3d(result[0, 3], result[1, 3], result[2, 3])
    
    # Create Affine3d using rotation matrices
    # Build it step by step using the rotation static methods
    affine = gk.Affine3d.identity()
    
    # Set the rotation by using the matrix elements
    # This is a workaround - we'll build using elementary rotations
    # For now, let's extract Euler angles and reconstruct
    # This is approximate but should work for the transform chain
    
    # Better approach: use the constructor with axes
    x_vec = gk.Vector3d(result[0, 0], result[1, 0], result[2, 0]).unitVector()
    y_vec = gk.Vector3d(result[0, 1], result[1, 1], result[2, 1]).unitVector()
    z_vec = gk.Vector3d(result[0, 2], result[1, 2], result[2, 2]).unitVector()
    
    affine = gk.Affine3d(x_vec, y_vec, z_vec, origin)
    
    return affine


class Trajectory:
    """Represents a surgical trajectory in DRB frame"""
    def __init__(self, name, start_drb, end_drb):
        self.name = name
        self.start_drb = np.array(start_drb)  # [x, y, z] in DRB frame
        self.end_drb = np.array(end_drb)      # [x, y, z] in DRB frame
    
    def to_dict(self):
        return {
            'name': self.name,
            'start_drb': self.start_drb.tolist(),
            'end_drb': self.end_drb.tolist()
        }
    
    @staticmethod
    def from_dict(d):
        return Trajectory(d['name'], d['start_drb'], d['end_drb'])
    
    def get_direction(self):
        """Get unit direction vector (end -> start)"""
        vec = self.start_drb - self.end_drb
        norm = np.linalg.norm(vec)
        if norm < 1e-6:
            return np.array([0, 0, 1])
        return vec / norm
    
    def interpolate_points(self, spacing_mm=20.0):
        """Get points along trajectory spaced by spacing_mm"""
        vec = self.start_drb - self.end_drb
        length = np.linalg.norm(vec)
        if length < 1e-6:
            return [self.end_drb.copy()]
        
        num_points = int(np.ceil(length / spacing_mm)) + 1
        points = []
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            point = self.end_drb + t * vec
            points.append(point)
        return points
    
class IKSimulatorApp(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Check if bindings are loaded
        global gk
        if gk is None:
            self.load_bindings()
        
        # Initialize IK solver components
        self.init_ik_solver()
        
        # Application state
        self.trajectories = []
        self.selected_trajectory_idx = None
        self.drb_position = np.array([500.0, 0.0, 200.0])  # Default DRB position
        self.drb_rotation = np.array([0.0, 0.0, 0.0])  # Roll, Pitch, Yaw in degrees
        self.ee_pitch = 10.0  # degrees
        self.ee_roll = 0.0    # degrees
        self.current_joints = None
        self.last_successful_joints = None
        
        # Trajectory solution storage and animation
        self.trajectory_solutions = []  # List of joint solutions along trajectory
        self.current_solution_idx = 0   # Current index in trajectory solutions
        self.is_playing = False          # Animation state
        
        # Initialize with default joints
        self.current_joints = gk.Joints(
            200.0,  # vertical_mm
            gk.Angle.fromDegrees(0.0),  # shoulder
            gk.Angle.fromDegrees(0.00001),  # elbow (avoid singularity)
            gk.Angle.fromDegrees(0.00001),  # roll (avoid singularity)
            gk.Angle.fromDegrees(0.0)   # pitch
        )
        self.last_successful_joints = self.current_joints
        
        # Visualization state
        self.prev_view = {"elev": 30, "azim": 30}
        self.ax = None
        
        # Reachability analysis state
        self.reachability_running = False
        self.reachability_cancelled = False

        # Hyperparam sweep state
        self.sweep_running = False
        self.sweep_cancelled = False
        self.sweep_paused = False
        self.sweep_config = None
        self.sweep_results = []
        
        # Animation timer
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.animation_step)
        self.animation_speed = 100  # milliseconds between frames
        
        self.init_ui()
        self.update_3d_plot()
        
    def load_bindings(self):
        """Load the GPS IK Python bindings - AUTO-LOAD from script directory"""
        global gk
        
        # Get script directory
        script_dir = Path(__file__).parent
        
        # Find .pyd file in script directory
        pyd_files = list(script_dir.glob("*.pyd"))
        
        if not pyd_files:
            QMessageBox.critical(
                None,
                "No Bindings Found",
                f"No .pyd file found in:\n{script_dir}\n\n"
                f"Please place the GPS IK Python bindings (.pyd file) in the same directory as this script."
            )
            sys.exit(1)
        
        if len(pyd_files) > 1:
            # Multiple .pyd files - let user choose
            pyd_names = [f.name for f in pyd_files]
            choice, ok = QInputDialog.getItem(
                None,
                "Multiple .pyd Files Found",
                "Select the GPS IK bindings file:",
                pyd_names,
                0,
                False
            )
            
            if not ok:
                sys.exit(1)
            
            pyd_path = script_dir / choice
        else:
            # Single .pyd file - use it automatically
            pyd_path = pyd_files[0]
        
        print(f"✅ Found bindings: {pyd_path.name}")
        
        # Get the directory containing the .pyd file
        pyd_dir = str(pyd_path.parent)
        
        # Add directory to sys.path for module import
        if pyd_dir not in sys.path:
            sys.path.insert(0, pyd_dir)
        
        # Add to DLL search path (Windows only)
        if sys.platform == 'win32':
            if hasattr(os, 'add_dll_directory'):
                os.add_dll_directory(pyd_dir)
            os.environ['PATH'] = pyd_dir + os.pathsep + os.environ.get('PATH', '')
        
        try:
            # Extract the base module name
            filename = pyd_path.name
            if filename.endswith('.pyd'):
                filename = filename[:-4]
            module_name = filename.split('.')[0]
            
            print(f"Importing module: {module_name}")
            
            import importlib
            gk_module = importlib.import_module(module_name)
            gk = gk_module
            
            print(f"✅ Successfully loaded: {pyd_path.name}")
            
        except Exception as e:
            print(f"❌ Failed to load bindings: {str(e)}")
            import traceback
            traceback.print_exc()
            
            QMessageBox.critical(
                None,
                "Failed to Load Bindings",
                f"Could not load Python bindings.\n\n"
                f"Error: {str(e)}\n\n"
                f"Module: {module_name}\n"
                f"File: {pyd_path}\n\n"
                f"See console for details."
            )
            sys.exit(1)
    
    def init_ik_solver(self):
        """Initialize IK solver components"""
        try:
            # Get script directory for INI file paths
            script_dir = Path(__file__).parent
            dh_config_path = str(script_dir / "DHConfig.ini")
            axis_config_path = str(script_dir / "config.ini")
            
            # Create DH config with absolute path
            dh_config = gk.DHConfigFromIniFactory.createInstance(dh_config_path)
            
            # Create axis config with absolute path
            axis_config = gk.AxisConfigFromIniFactory.createInstance(axis_config_path)
            
            # Create forward kinematics
            self.fk = gk.ForwardKinematicsGps(dh_config)
            
            # Create axis config group
            self.axis_config_group = gk.AxisConfigGroup(axis_config)
            
            # Create validator
            self.validator = gk.GpsIKSolutionValidator(self.fk, self.axis_config_group)
            
            # Create IK solver
            self.ik_solver = gk.InverseKinematicsGps(self.fk)
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to initialize IK solver:\n{str(e)}")
            sys.exit(1)
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Eflex Base Position Helper')
        self.setGeometry(100, 100, 1400, 900)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel - DRB controls and trajectory management
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel, stretch=1)
        
        # Center panel - 3D visualization
        center_panel = self.create_center_panel()
        main_layout.addWidget(center_panel, stretch=3)
        
        # Right panel - Joint angles and controls
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, stretch=1)
        
        # Status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('Ready')

    def create_left_panel(self):
        """Create left panel with DRB controls and trajectory management"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # DRB Position Group
        drb_pos_group = QGroupBox("DRB Position (mm)")
        drb_pos_layout = QVBoxLayout()
        
        self.drb_pos_controls = {}
        for axis, (min_val, max_val) in [('X', (-1000, 1000)), 
                                          ('Y', (-1000, 1500)), 
                                          ('Z', (-200, 500))]:
            h_layout = QHBoxLayout()
            h_layout.addWidget(QLabel(f"{axis}:"))
            
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(int(min_val))
            slider.setMaximum(int(max_val))
            slider.setValue(int(self.drb_position[['X', 'Y', 'Z'].index(axis)]))
            slider.valueChanged.connect(lambda v, a=axis: self.on_drb_pos_changed(a, v))
            
            text = QLineEdit(str(self.drb_position[['X', 'Y', 'Z'].index(axis)]))
            text.setMaximumWidth(80)
            text.returnPressed.connect(lambda a=axis, t=text: self.on_drb_pos_text_changed(a, t))
            
            self.drb_pos_controls[axis] = {'slider': slider, 'text': text}
            
            h_layout.addWidget(slider)
            h_layout.addWidget(text)
            drb_pos_layout.addLayout(h_layout)
        
        drb_pos_group.setLayout(drb_pos_layout)
        layout.addWidget(drb_pos_group)
        
        # DRB Rotation Group
        drb_rot_group = QGroupBox("DRB Rotation (degrees)")
        drb_rot_layout = QVBoxLayout()
        
        self.drb_rot_controls = {}
        for axis in ['Roll', 'Pitch', 'Yaw']:
            h_layout = QHBoxLayout()
            h_layout.addWidget(QLabel(f"{axis}:"))
            
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(0)
            slider.valueChanged.connect(lambda v, a=axis: self.on_drb_rot_changed(a, v))
            
            text = QLineEdit("0.0")
            text.setMaximumWidth(80)
            text.returnPressed.connect(lambda a=axis, t=text: self.on_drb_rot_text_changed(a, t))
            
            self.drb_rot_controls[axis] = {'slider': slider, 'text': text}
            
            h_layout.addWidget(slider)
            h_layout.addWidget(text)
            drb_rot_layout.addLayout(h_layout)
        
        drb_rot_group.setLayout(drb_rot_layout)
        layout.addWidget(drb_rot_group)
        
        # Trajectory Management Group
        traj_group = QGroupBox("Trajectories")
        traj_layout = QVBoxLayout()
        
        # Trajectory list
        self.trajectory_list = QListWidget()
        self.trajectory_list.itemSelectionChanged.connect(self.on_trajectory_selected)
        traj_layout.addWidget(self.trajectory_list)
        
        # Add/Remove/Rename buttons
        btn_layout = QHBoxLayout()
        self.add_traj_btn = QPushButton("Add")
        self.add_traj_btn.clicked.connect(self.add_trajectory)
        self.remove_traj_btn = QPushButton("Remove")
        self.remove_traj_btn.clicked.connect(self.remove_trajectory)
        self.rename_traj_btn = QPushButton("Rename")
        self.rename_traj_btn.clicked.connect(self.rename_trajectory)
        
        btn_layout.addWidget(self.add_traj_btn)
        btn_layout.addWidget(self.remove_traj_btn)
        btn_layout.addWidget(self.rename_traj_btn)
        traj_layout.addLayout(btn_layout)
        
        traj_group.setLayout(traj_layout)
        layout.addWidget(traj_group)
        
        # Trajectory point adjustment - Combined buttons + text boxes
        adj_group = QGroupBox("Trajectory Point Adjustment")
        adj_layout = QVBoxLayout()
        
        # Start point controls
        start_label = QLabel("Start Point (DRB frame, mm):")
        start_label.setStyleSheet("font-weight: bold;")
        adj_layout.addWidget(start_label)
        
        # Start X
        start_x_layout = QHBoxLayout()
        start_x_layout.addWidget(QLabel("X:"))
        self.start_x_text = QLineEdit("0.0")
        self.start_x_text.setMaximumWidth(100)
        self.start_x_text.returnPressed.connect(self.on_trajectory_coord_changed)
        start_x_layout.addWidget(self.start_x_text)
        
        start_x_minus = QPushButton("-")
        start_x_minus.setMaximumWidth(30)
        start_x_minus.clicked.connect(lambda: self.adjust_trajectory_point('start', 'x', -10))
        start_x_layout.addWidget(start_x_minus)
        
        start_x_plus = QPushButton("+")
        start_x_plus.setMaximumWidth(30)
        start_x_plus.clicked.connect(lambda: self.adjust_trajectory_point('start', 'x', 10))
        start_x_layout.addWidget(start_x_plus)
        
        adj_layout.addLayout(start_x_layout)
        
        # Start Y
        start_y_layout = QHBoxLayout()
        start_y_layout.addWidget(QLabel("Y:"))
        self.start_y_text = QLineEdit("0.0")
        self.start_y_text.setMaximumWidth(100)
        self.start_y_text.returnPressed.connect(self.on_trajectory_coord_changed)
        start_y_layout.addWidget(self.start_y_text)
        
        start_y_minus = QPushButton("-")
        start_y_minus.setMaximumWidth(30)
        start_y_minus.clicked.connect(lambda: self.adjust_trajectory_point('start', 'y', -10))
        start_y_layout.addWidget(start_y_minus)
        
        start_y_plus = QPushButton("+")
        start_y_plus.setMaximumWidth(30)
        start_y_plus.clicked.connect(lambda: self.adjust_trajectory_point('start', 'y', 10))
        start_y_layout.addWidget(start_y_plus)
        
        adj_layout.addLayout(start_y_layout)
        
        # Start Z
        start_z_layout = QHBoxLayout()
        start_z_layout.addWidget(QLabel("Z:"))
        self.start_z_text = QLineEdit("200.0")
        self.start_z_text.setMaximumWidth(100)
        self.start_z_text.returnPressed.connect(self.on_trajectory_coord_changed)
        start_z_layout.addWidget(self.start_z_text)
        
        start_z_minus = QPushButton("-")
        start_z_minus.setMaximumWidth(30)
        start_z_minus.clicked.connect(lambda: self.adjust_trajectory_point('start', 'z', -10))
        start_z_layout.addWidget(start_z_minus)
        
        start_z_plus = QPushButton("+")
        start_z_plus.setMaximumWidth(30)
        start_z_plus.clicked.connect(lambda: self.adjust_trajectory_point('start', 'z', 10))
        start_z_layout.addWidget(start_z_plus)
        
        adj_layout.addLayout(start_z_layout)
        
        # Separator
        adj_layout.addWidget(QLabel(""))
        
        # End point controls
        end_label = QLabel("End Point (DRB frame, mm):")
        end_label.setStyleSheet("font-weight: bold;")
        adj_layout.addWidget(end_label)
        
        # End X
        end_x_layout = QHBoxLayout()
        end_x_layout.addWidget(QLabel("X:"))
        self.end_x_text = QLineEdit("0.0")
        self.end_x_text.setMaximumWidth(100)
        self.end_x_text.returnPressed.connect(self.on_trajectory_coord_changed)
        end_x_layout.addWidget(self.end_x_text)
        
        end_x_minus = QPushButton("-")
        end_x_minus.setMaximumWidth(30)
        end_x_minus.clicked.connect(lambda: self.adjust_trajectory_point('end', 'x', -10))
        end_x_layout.addWidget(end_x_minus)
        
        end_x_plus = QPushButton("+")
        end_x_plus.setMaximumWidth(30)
        end_x_plus.clicked.connect(lambda: self.adjust_trajectory_point('end', 'x', 10))
        end_x_layout.addWidget(end_x_plus)
        
        adj_layout.addLayout(end_x_layout)
        
        # End Y
        end_y_layout = QHBoxLayout()
        end_y_layout.addWidget(QLabel("Y:"))
        self.end_y_text = QLineEdit("0.0")
        self.end_y_text.setMaximumWidth(100)
        self.end_y_text.returnPressed.connect(self.on_trajectory_coord_changed)
        end_y_layout.addWidget(self.end_y_text)
        
        end_y_minus = QPushButton("-")
        end_y_minus.setMaximumWidth(30)
        end_y_minus.clicked.connect(lambda: self.adjust_trajectory_point('end', 'y', -10))
        end_y_layout.addWidget(end_y_minus)
        
        end_y_plus = QPushButton("+")
        end_y_plus.setMaximumWidth(30)
        end_y_plus.clicked.connect(lambda: self.adjust_trajectory_point('end', 'y', 10))
        end_y_layout.addWidget(end_y_plus)
        
        adj_layout.addLayout(end_y_layout)
        
        # End Z
        end_z_layout = QHBoxLayout()
        end_z_layout.addWidget(QLabel("Z:"))
        self.end_z_text = QLineEdit("100.0")
        self.end_z_text.setMaximumWidth(100)
        self.end_z_text.returnPressed.connect(self.on_trajectory_coord_changed)
        end_z_layout.addWidget(self.end_z_text)
        
        end_z_minus = QPushButton("-")
        end_z_minus.setMaximumWidth(30)
        end_z_minus.clicked.connect(lambda: self.adjust_trajectory_point('end', 'z', -10))
        end_z_layout.addWidget(end_z_minus)
        
        end_z_plus = QPushButton("+")
        end_z_plus.setMaximumWidth(30)
        end_z_plus.clicked.connect(lambda: self.adjust_trajectory_point('end', 'z', 10))
        end_z_layout.addWidget(end_z_plus)
        
        adj_layout.addLayout(end_z_layout)
        
        # Apply coordinates button
        apply_coords_btn = QPushButton("Apply Coordinates")
        apply_coords_btn.clicked.connect(self.apply_trajectory_coordinates)
        adj_layout.addWidget(apply_coords_btn)
        
        adj_group.setLayout(adj_layout)
        layout.addWidget(adj_group)
        
        # File operations
        file_group = QGroupBox("File Operations")
        file_layout = QVBoxLayout()
        
        save_btn = QPushButton("Save Configuration")
        save_btn.clicked.connect(self.save_configuration)
        load_btn = QPushButton("Load Configuration")
        load_btn.clicked.connect(self.load_configuration)
        
        file_layout.addWidget(save_btn)
        file_layout.addWidget(load_btn)
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        layout.addStretch()
        return panel
    
    def create_center_panel(self):
        """Create center panel with 3D visualization"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Create matplotlib figure
        self.figure = Figure(figsize=(10, 8))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')

        self.ax.set_xlim([-1000, 1000])
        self.ax.set_ylim([-1000, 1500])
        self.ax.set_zlim([-400, 1000])
        
        # Add toolbar
        self.toolbar = NavigationToolbar(self.canvas, panel)
        
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        
        return panel
    
    def create_right_panel(self):
        """Create right panel with controls and joint display"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # EE Orientation Group
        ee_group = QGroupBox("End Effector Orientation")
        ee_layout = QVBoxLayout()
        
        # Pitch control
        pitch_layout = QHBoxLayout()
        pitch_layout.addWidget(QLabel("Pitch (°):"))
        self.ee_pitch_slider = QSlider(Qt.Orientation.Horizontal)
        self.ee_pitch_slider.setMinimum(-180)
        self.ee_pitch_slider.setMaximum(180)
        self.ee_pitch_slider.setValue(int(self.ee_pitch))
        self.ee_pitch_slider.valueChanged.connect(self.on_ee_pitch_changed)
        
        self.ee_pitch_text = QLineEdit(str(self.ee_pitch))
        self.ee_pitch_text.setMaximumWidth(80)
        self.ee_pitch_text.returnPressed.connect(self.on_ee_pitch_text_changed)
        
        pitch_layout.addWidget(self.ee_pitch_slider)
        pitch_layout.addWidget(self.ee_pitch_text)
        ee_layout.addLayout(pitch_layout)
        
        # Roll control
        roll_layout = QHBoxLayout()
        roll_layout.addWidget(QLabel("Roll (°):"))
        self.ee_roll_slider = QSlider(Qt.Orientation.Horizontal)
        self.ee_roll_slider.setMinimum(-180)
        self.ee_roll_slider.setMaximum(180)
        self.ee_roll_slider.setValue(int(self.ee_roll))
        self.ee_roll_slider.valueChanged.connect(self.on_ee_roll_changed)
        
        self.ee_roll_text = QLineEdit(str(self.ee_roll))
        self.ee_roll_text.setMaximumWidth(80)
        self.ee_roll_text.returnPressed.connect(self.on_ee_roll_text_changed)
        
        roll_layout.addWidget(self.ee_roll_slider)
        roll_layout.addWidget(self.ee_roll_text)
        ee_layout.addLayout(roll_layout)
        
        ee_group.setLayout(ee_layout)
        layout.addWidget(ee_group)
        
        # Manual Joint Control Group - NEW!
        joint_control_group = QGroupBox("Manual Joint Control")
        joint_control_layout = QVBoxLayout()
        
        self.joint_controls = {}
        
        # Vertical joint (linear, mm)
        v_layout = QHBoxLayout()
        v_layout.addWidget(QLabel("Vertical (mm):"))
        v_text = QLineEdit("200.0")
        v_text.setMaximumWidth(100)
        v_text.returnPressed.connect(lambda: self.on_manual_joint_changed('vertical', v_text))
        self.joint_controls['vertical'] = v_text
        v_layout.addWidget(v_text)
        joint_control_layout.addLayout(v_layout)
        
        # Shoulder joint (degrees)
        s_layout = QHBoxLayout()
        s_layout.addWidget(QLabel("Shoulder (°):"))
        s_text = QLineEdit("0.0")
        s_text.setMaximumWidth(100)
        s_text.returnPressed.connect(lambda: self.on_manual_joint_changed('shoulder', s_text))
        self.joint_controls['shoulder'] = s_text
        s_layout.addWidget(s_text)
        joint_control_layout.addLayout(s_layout)
        
        # Elbow joint (degrees)
        e_layout = QHBoxLayout()
        e_layout.addWidget(QLabel("Elbow (°):"))
        e_text = QLineEdit("0.1")
        e_text.setMaximumWidth(100)
        e_text.returnPressed.connect(lambda: self.on_manual_joint_changed('elbow', e_text))
        self.joint_controls['elbow'] = e_text
        e_layout.addWidget(e_text)
        joint_control_layout.addLayout(e_layout)
        
        # Roll joint (degrees)
        r_layout = QHBoxLayout()
        r_layout.addWidget(QLabel("Roll (°):"))
        r_text = QLineEdit("-0.1")
        r_text.setMaximumWidth(100)
        r_text.returnPressed.connect(lambda: self.on_manual_joint_changed('roll', r_text))
        self.joint_controls['roll'] = r_text
        r_layout.addWidget(r_text)
        joint_control_layout.addLayout(r_layout)
        
        # Pitch joint (degrees)
        p_layout = QHBoxLayout()
        p_layout.addWidget(QLabel("Pitch (°):"))
        p_text = QLineEdit("0.0")
        p_text.setMaximumWidth(100)
        p_text.returnPressed.connect(lambda: self.on_manual_joint_changed('pitch', p_text))
        self.joint_controls['pitch'] = p_text
        p_layout.addWidget(p_text)
        joint_control_layout.addLayout(p_layout)
        
        # Apply button
        apply_joints_btn = QPushButton("Apply Joint Angles")
        apply_joints_btn.clicked.connect(self.apply_manual_joints)
        joint_control_layout.addWidget(apply_joints_btn)
        
        # Reset to home button
        reset_joints_btn = QPushButton("Reset to Home")
        reset_joints_btn.clicked.connect(self.reset_joints_to_home)
        joint_control_layout.addWidget(reset_joints_btn)
        
        joint_control_group.setLayout(joint_control_layout)
        layout.addWidget(joint_control_group)
        
        # Joint Angles Display Group (read-only, shows current FK result)
        joints_group = QGroupBox("Current Joint Angles (FK)")
        joints_layout = QVBoxLayout()
        
        self.joint_labels = {}
        for joint_name in ['Vertical', 'Shoulder', 'Elbow', 'Roll', 'Pitch']:
            h_layout = QHBoxLayout()
            h_layout.addWidget(QLabel(f"{joint_name}:"))
            label = QLabel("N/A")
            label.setMinimumWidth(100)
            self.joint_labels[joint_name] = label
            h_layout.addWidget(label)
            h_layout.addStretch()
            joints_layout.addLayout(h_layout)
        
        # Solve time
        solve_time_layout = QHBoxLayout()
        solve_time_layout.addWidget(QLabel("Solve Time:"))
        self.solve_time_label = QLabel("N/A")
        self.solve_time_label.setMinimumWidth(100)
        solve_time_layout.addWidget(self.solve_time_label)
        solve_time_layout.addStretch()
        joints_layout.addLayout(solve_time_layout)
        
        joints_group.setLayout(joints_layout)
        layout.addWidget(joints_group)
        
        # IK Solve Group
        ik_group = QGroupBox("IK Solver")
        ik_layout = QVBoxLayout()
        
        self.solve_ik_btn = QPushButton("Solve IK for Selected Trajectory")
        self.solve_ik_btn.clicked.connect(self.solve_ik_for_trajectory)
        self.solve_ik_btn.setEnabled(False)
        ik_layout.addWidget(self.solve_ik_btn)
        
        # Animation controls
        anim_layout = QHBoxLayout()
        
        self.play_pause_btn = QPushButton("Play")
        self.play_pause_btn.clicked.connect(self.toggle_play_pause)
        self.play_pause_btn.setEnabled(False)
        anim_layout.addWidget(self.play_pause_btn)
        
        self.prev_solution_btn = QPushButton("<-")
        self.prev_solution_btn.clicked.connect(self.prev_trajectory_solution)
        self.prev_solution_btn.setEnabled(False)
        anim_layout.addWidget(self.prev_solution_btn)
        
        self.next_solution_btn = QPushButton("->")
        self.next_solution_btn.clicked.connect(self.next_trajectory_solution)
        self.next_solution_btn.setEnabled(False)
        anim_layout.addWidget(self.next_solution_btn)
        
        ik_layout.addLayout(anim_layout)
        
        # Solution index label
        self.solution_idx_label = QLabel("Solution: N/A")
        ik_layout.addWidget(self.solution_idx_label)
        
        # Speed control
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setMinimum(10)
        self.speed_slider.setMaximum(1000)
        self.speed_slider.setValue(100)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        speed_layout.addWidget(self.speed_slider)
        self.speed_label = QLabel("100ms")
        speed_layout.addWidget(self.speed_label)
        ik_layout.addLayout(speed_layout)
        
        ik_group.setLayout(ik_layout)
        layout.addWidget(ik_group)
        
        # Reachability Analysis Group
        reach_group = QGroupBox("Reachability Analysis")
        reach_layout = QVBoxLayout()
        
        self.analyze_reach_btn = QPushButton("Analyze Reachability")
        self.analyze_reach_btn.clicked.connect(self.analyze_reachability)
        reach_layout.addWidget(self.analyze_reach_btn)
        
        self.reach_progress = QProgressBar()
        self.reach_progress.setVisible(False)
        reach_layout.addWidget(self.reach_progress)
        
        self.cancel_reach_btn = QPushButton("Cancel")
        self.cancel_reach_btn.clicked.connect(self.cancel_reachability)
        self.cancel_reach_btn.setVisible(False)
        reach_layout.addWidget(self.cancel_reach_btn)
        
        reach_group.setLayout(reach_layout)
        layout.addWidget(reach_group)
        
        # Hyperparameter Sweep Group
        sweep_group = QGroupBox("Hyperparameter Sweep")
        sweep_layout = QVBoxLayout()
        
        self.start_sweep_btn = QPushButton("don't do this")
        self.start_sweep_btn.clicked.connect(self.start_hyperparam_sweep)
        sweep_layout.addWidget(self.start_sweep_btn)
        
        self.sweep_progress = QProgressBar()
        self.sweep_progress.setVisible(False)
        sweep_layout.addWidget(self.sweep_progress)
        
        # Sweep control buttons
        sweep_control_layout = QHBoxLayout()
        self.pause_sweep_btn = QPushButton("Pause")
        self.pause_sweep_btn.clicked.connect(self.pause_hyperparam_sweep)
        self.pause_sweep_btn.setVisible(False)
        sweep_control_layout.addWidget(self.pause_sweep_btn)
        
        self.cancel_sweep_btn = QPushButton("Cancel")
        self.cancel_sweep_btn.clicked.connect(self.cancel_hyperparam_sweep)
        self.cancel_sweep_btn.setVisible(False)
        sweep_control_layout.addWidget(self.cancel_sweep_btn)
        
        sweep_layout.addLayout(sweep_control_layout)
        
        self.sweep_status_label = QLabel("")
        sweep_layout.addWidget(self.sweep_status_label)
        
        sweep_group.setLayout(sweep_layout)
        layout.addWidget(sweep_group)
        
        layout.addStretch()
        
        # Update initial joint display
        self.update_joint_display()
        
        return panel

    # Manual Joint Control Methods
    def on_manual_joint_changed(self, joint_name, text_widget):
        """Handle manual joint text input change (triggered on Enter key)"""
        try:
            value = float(text_widget.text())
            
            # Update the text widget with formatted value
            if joint_name == 'vertical':
                text_widget.setText(f"{value:.2f}")
            else:
                text_widget.setText(f"{value:.2f}")
            
            self.statusBar.showMessage(f"Updated {joint_name} to {value}")
            
        except ValueError:
            self.statusBar.showMessage(f"Invalid value for {joint_name}")
            # Reset to current value
            if self.current_joints:
                if joint_name == 'vertical':
                    text_widget.setText(f"{self.current_joints.vertical:.2f}")
                elif joint_name == 'shoulder':
                    text_widget.setText(f"{self.current_joints.shoulder.degrees():.2f}")
                elif joint_name == 'elbow':
                    text_widget.setText(f"{self.current_joints.elbow.degrees():.2f}")
                elif joint_name == 'roll':
                    text_widget.setText(f"{self.current_joints.roll.degrees():.2f}")
                elif joint_name == 'pitch':
                    text_widget.setText(f"{self.current_joints.pitch.degrees():.2f}")
    
    def apply_manual_joints(self):
        """Apply manually entered joint angles and update visualization"""
        try:
            # Read values from text boxes
            vertical = float(self.joint_controls['vertical'].text())
            shoulder = float(self.joint_controls['shoulder'].text())
            elbow = float(self.joint_controls['elbow'].text())
            roll = float(self.joint_controls['roll'].text())
            pitch = float(self.joint_controls['pitch'].text())
            
            # Validate ranges (optional)
            # You can add axis limit checks here based on config.ini
            
            # Create new joint configuration
            self.current_joints = gk.Joints(
                vertical,
                gk.Angle.fromDegrees(shoulder),
                gk.Angle.fromDegrees(elbow),
                gk.Angle.fromDegrees(roll),
                gk.Angle.fromDegrees(pitch)
            )
            
            # Update last successful joints
            self.last_successful_joints = self.current_joints
            
            # Update displays
            self.update_joint_display()
            self.update_3d_plot()
            
            self.statusBar.showMessage(
                f"Applied manual joints: V={vertical:.1f}mm, S={shoulder:.1f}°, "
                f"E={elbow:.1f}°, R={roll:.1f}°, P={pitch:.1f}°"
            )
            
        except ValueError as e:
            QMessageBox.warning(self, "Invalid Input", f"Please enter valid numbers for all joints.\n\nError: {str(e)}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to apply joint angles:\n{str(e)}")
    
    def reset_joints_to_home(self):
        """Reset joints to home position"""
        # Home position
        self.current_joints = gk.Joints(
            200.0,  # vertical at 200mm
            gk.Angle.fromDegrees(0.0),
            gk.Angle.fromDegrees(0.1),  # Small non-zero to avoid singularity
            gk.Angle.fromDegrees(-0.1),
            gk.Angle.fromDegrees(0.0)
        )
        
        # Update manual control text boxes
        self.joint_controls['vertical'].setText("200.0")
        self.joint_controls['shoulder'].setText("0.0")
        self.joint_controls['elbow'].setText("0.1")
        self.joint_controls['roll'].setText("-0.1")
        self.joint_controls['pitch'].setText("0.0")
        
        # Update displays
        self.update_joint_display()
        self.update_3d_plot()
        
        self.statusBar.showMessage("Reset to home position")
    
        # Hyperparameter Sweep Methods
    def start_hyperparam_sweep(self):
        """Start hyperparameter sweep - load config and begin"""
        if not self.trajectories:
            QMessageBox.warning(self, "Warning", "No trajectories defined")
            return
        
        # Ask user to select sweep configuration JSON
        script_dir = str(Path(__file__).parent)
        config_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Hyperparam Sweep Configuration",
            script_dir,
            "JSON Files (*.json);;All Files (*)"
        )
        
        if not config_path:
            return
        
        # Load configuration
        try:
            with open(config_path, 'r') as f:
                self.sweep_config = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load config:\n{str(e)}")
            return
        
        # Validate configuration
        required_keys = ['ee_roll_values', 'ee_pitch_values', 
                        'drb_rotations_roll', 'drb_rotations_pitch', 'drb_rotations_yaw',
                        'handedness']
        for key in required_keys:
            if key not in self.sweep_config:
                QMessageBox.critical(self, "Error", f"Missing required key in config: {key}")
                return
        
        # Create output directory with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.sweep_output_dir = Path(script_dir) / f"hyperparam_sweep_{timestamp}"
        self.sweep_output_dir.mkdir(exist_ok=True)
        
        # Calculate total configurations
        total_configs = (
            len(self.sweep_config['ee_roll_values']) *
            len(self.sweep_config['ee_pitch_values']) *
            len(self.sweep_config['drb_rotations_roll']) *
            len(self.sweep_config['drb_rotations_pitch']) *
            len(self.sweep_config['drb_rotations_yaw']) *
            len(self.sweep_config['handedness'])
        )
        
        # Confirm with user
        msg = (
            f"Hyperparam Sweep Configuration:\n\n"
            f"EE Roll values: {len(self.sweep_config['ee_roll_values'])}\n"
            f"EE Pitch values: {len(self.sweep_config['ee_pitch_values'])}\n"
            f"DRB Rotation Roll: {len(self.sweep_config['drb_rotations_roll'])}\n"
            f"DRB Rotation Pitch: {len(self.sweep_config['drb_rotations_pitch'])}\n"
            f"DRB Rotation Yaw: {len(self.sweep_config['drb_rotations_yaw'])}\n"
            f"Handedness modes: {len(self.sweep_config['handedness'])}\n\n"
            f"Total configurations: {total_configs}\n"
            f"Trajectories to test: {len(self.trajectories)}\n\n"
            f"Output directory: {self.sweep_output_dir}\n\n"
            f"This will take several hours. Continue?"
        )
        
        reply = QMessageBox.question(self, "Confirm Sweep", msg,
                                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        # Initialize sweep state
        self.sweep_running = True
        self.sweep_cancelled = False
        self.sweep_paused = False
        self.sweep_results = []
        
        # Show progress UI
        self.sweep_progress.setVisible(True)
        self.sweep_progress.setMaximum(total_configs)
        self.sweep_progress.setValue(0)
        self.pause_sweep_btn.setVisible(True)
        self.cancel_sweep_btn.setVisible(True)
        self.start_sweep_btn.setEnabled(False)
        self.analyze_reach_btn.setEnabled(False)
        
        # Run sweep
        self.run_hyperparam_sweep()
    
    def run_hyperparam_sweep(self):
        """Execute the hyperparameter sweep"""
        # Generate all configurations
        configs = []
        baseline_config = {
            'ee_roll': 0.0,
            'ee_pitch': 10.0,
        }
        for drb_rot_roll in self.sweep_config['drb_rotations_roll']:
            for drb_rot_pitch in self.sweep_config['drb_rotations_pitch']:
                for drb_rot_yaw in self.sweep_config['drb_rotations_yaw']:
                    for handedness in self.sweep_config['handedness']:
                        for ee_roll in self.sweep_config['ee_roll_values']:
                            configs.append({
                                        'ee_roll': ee_roll,
                                        'ee_pitch': baseline_config['ee_pitch'],
                                        'drb_rotation': [drb_rot_roll, drb_rot_pitch, drb_rot_yaw],
                                        'handedness': handedness
                            })
                        for ee_pitch in self.sweep_config['ee_pitch_values']:
                                configs.append({
                                    'ee_roll': baseline_config['ee_roll'],
                                    'ee_pitch': ee_pitch,
                                    'drb_rotation': [drb_rot_roll, drb_rot_pitch, drb_rot_yaw],
                                    'handedness': handedness
                                })
        
        # Process each configuration
        for config_idx, config in enumerate(configs):
            if self.sweep_cancelled:
                break
            
            # Handle pause
            while self.sweep_paused and not self.sweep_cancelled:
                QApplication.processEvents()
                time.sleep(0.1)
            
            if self.sweep_cancelled:
                break
            
            # Update status
            self.sweep_status_label.setText(
                f"Config {config_idx + 1}/{len(configs)}: "
                f"EE_roll={config['ee_roll']}°, EE_pitch={config['ee_pitch']}°, "
                f"DRB_rot={config['drb_rotation']}, "
                f"Hand={'Right' if config['handedness'] else 'Left'}"
            )
            QApplication.processEvents()
            
            # Run reachability analysis for this configuration
            result = self.analyze_single_configuration(config)
            
            if result is not None:
                # Save result
                self.sweep_results.append(result)
                
                # ALWAYS save plot (even if count == 0)
                self.save_sweep_plot(result, config_idx + 1, len(configs))
            
            # Update progress
            self.sweep_progress.setValue(config_idx + 1)
            QApplication.processEvents()
        
        # Cleanup
        self.sweep_progress.setVisible(False)
        self.pause_sweep_btn.setVisible(False)
        self.cancel_sweep_btn.setVisible(False)
        self.start_sweep_btn.setEnabled(True)
        self.analyze_reach_btn.setEnabled(True)
        self.sweep_running = False
        
        if self.sweep_cancelled:
            self.statusBar.showMessage("Hyperparam sweep cancelled")
            self.sweep_status_label.setText("Cancelled")
        else:
            # Save summary table
            self.save_sweep_summary()
            self.statusBar.showMessage(f"Hyperparam sweep complete! Results in {self.sweep_output_dir}")
            self.sweep_status_label.setText("Complete")
            QMessageBox.information(self, "Sweep Complete",
                                  f"Processed {len(configs)} configurations.\n"
                                  f"Results saved to:\n{self.sweep_output_dir}")
    
    def analyze_single_configuration(self, config):
        """Analyze reachability for a single hyperparameter configuration
        
        Tests which DRB positions allow the entire trajectory to be reached
        with this specific EE orientation, DRB rotation, and handedness.
        """
        ee_roll = config['ee_roll']
        ee_pitch = config['ee_pitch']
        drb_rot = np.array(config['drb_rotation'])  # [roll, pitch, yaw]
        handedness = config['handedness']
        
        # Define DRB position grid
        x_range = np.arange(-1000, 1001, 50)
        y_range = np.arange(-1000, 1501, 50)
        z_range = np.arange(-200, 501, 50)
        
        # Create robot_tool_x_ee_tip for this configuration
        robot_tool_x_ee_tip = compute_robot_tool_to_ee_tip(ee_roll, ee_pitch)
        
        # Initial joints based on handedness
        initial_joints = gk.Joints(
            200.0,
            gk.Angle.fromDegrees(0.0),
            gk.Angle.fromDegrees(0.1 if handedness else -0.1),
            gk.Angle.fromDegrees(-0.1),
            gk.Angle.fromDegrees(0.0)
        )
        
        valid_drb_positions = []
        
        # Save current app state
        saved_drb_pos = self.drb_position.copy()
        saved_drb_rot = self.drb_rotation.copy()
        
        # Set DRB rotation for this configuration (stays constant)
        self.drb_rotation = drb_rot
        
        # Test each DRB position in the grid
        for drb_x in x_range:
            for drb_y in y_range:
                for drb_z in z_range:
                    if self.sweep_cancelled:
                        self.drb_position = saved_drb_pos
                        self.drb_rotation = saved_drb_rot
                        return None
                    
                    # Set DRB position for this test
                    self.drb_position = np.array([drb_x, drb_y, drb_z])
                    
                    # Test if ALL trajectories are reachable from this DRB position
                    all_trajectories_reachable = True
                    
                    for traj in self.trajectories:
                        # Get trajectory path points (in DRB frame)
                        trajectory_points_drb = traj.interpolate_points(spacing_mm=20.0)
                        
                        current_joints = initial_joints
                        trajectory_reachable = True
                        
                        for point_drb in trajectory_points_drb:
                            # Transform trajectory point from DRB frame to world frame
                            point_world = self.transform_point_drb_to_world(point_drb)
                            
                            # Get trajectory direction (from the trajectory itself)
                            direction_drb = traj.get_direction()
                            
                            # Transform direction from DRB frame to world frame
                            T_drb = self.get_drb_transform()
                            direction_world = T_drb[:3, :3] @ direction_drb
                            direction_world = direction_world / np.linalg.norm(direction_world)
                            
                            # Create target ray
                            target_point = gk.Point3d(point_world[0], point_world[1], point_world[2])
                            target_direction = gk.Vector3d(direction_world[0], direction_world[1], direction_world[2]).unitVector()
                            target_ray = gk.Ray3d(target_point, target_direction)
                            
                            # Solve IK for this point
                            success, solution = self.ik_solver.findSolutionQuick(
                                target_ray,
                                robot_tool_x_ee_tip,
                                current_joints,
                                self.validator,
                                wristMode=False,
                                rightHand=handedness,
                                tolerance=0.01
                            )
                            
                            if not success:
                                trajectory_reachable = False
                                break
                            
                            # Update for next point
                            current_joints = solution
                        
                        if not trajectory_reachable:
                            all_trajectories_reachable = False
                            break
                    
                    # If ALL trajectory points are reachable, record this DRB position
                    if all_trajectories_reachable:
                        valid_drb_positions.append([drb_x, drb_y, drb_z])
        
        # Restore app state
        self.drb_position = saved_drb_pos
        self.drb_rotation = saved_drb_rot
        
        return {
            'ee_roll': ee_roll,
            'ee_pitch': ee_pitch,
            'drb_rotation': drb_rot.tolist(),
            'handedness': handedness,
            'reachable_positions': valid_drb_positions,
            'count': len(valid_drb_positions)
        }
    
    def save_sweep_plot(self, result, current_config, total_configs):
        """Save 3D scatter plot for a single configuration (XY plane view)
        
        Saves plot even if no reachable positions (empty workspace visualization)
        """
        if result is None:
            return
        
        from matplotlib.patches import Circle
        from mpl_toolkits.mplot3d import art3d
        
        # Create figure
        fig = Figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Get base offset from FK
        base_offset = self.fk.getBaseOffsets()
        base_center = np.array([base_offset.x(), base_offset.y(), base_offset.z()])
        
        # Draw robot base cuboid - PASS ax parameter
        self.draw_robot_base_cube(base_center, ax=ax)
    
        # Plot reachable DRB positions (if any)
        if result['count'] > 0:
            positions = np.array(result['reachable_positions'])
            ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
                      c='lime', marker='.', s=5, alpha=0.5)
        
        # Set view to XY plane (looking down Z axis)
        ax.view_init(elev=90, azim=-90)
        
        # Labels
        ax.set_xlabel('X (mm)', fontsize=12)
        ax.set_ylabel('Y (mm)', fontsize=12)
        ax.set_zlabel('Z (mm)', fontsize=12)
        
        # Title - LARGER FONT
        hand_str = "Right Hand" if result['handedness'] else "Left Hand"
        drb_rot_str = f"[{result['drb_rotation'][0]:.0f}°, {result['drb_rotation'][1]:.0f}°, {result['drb_rotation'][2]:.0f}°]"
        
        # Add warning to title if no reachable positions
        status = f"Reachable Positions: {result['count']}"
        if result['count'] == 0:
            status = "⚠️ NO REACHABLE POSITIONS ⚠️"
        
        title = (f"EE Roll = {result['ee_roll']}°  |  EE Pitch = {result['ee_pitch']}°  |  {hand_str}\n"
                f"DRB Rotation = {drb_rot_str}\n"
                f"{status}")
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        
        # Set equal aspect ratio
        ax.set_box_aspect([1, 1, 1])
        
        # FIXED axis limits (same for all plots)
        ax.set_xlim([-1000, 1000])
        ax.set_ylim([-1000, 1500])
        ax.set_zlim([-200, 500])
        
        # Save
        filename = (f"config_{current_config:04d}_"
                   f"ee_roll{result['ee_roll']:+04.0f}_"
                   f"ee_pitch{result['ee_pitch']:+04.0f}_"
                   f"drb_rot{result['drb_rotation'][0]:.0f}_{result['drb_rotation'][1]:.0f}_{result['drb_rotation'][2]:.0f}_"
                   f"{'right' if result['handedness'] else 'left'}.png")
        
        filepath = self.sweep_output_dir / filename
        
        canvas = FigureCanvas(fig)
        canvas.print_figure(filepath, dpi=150, bbox_inches='tight')
        
        # Close figure to free memory
        plt.close(fig)
    
    def save_sweep_summary(self):
        """Save summary table of all configurations to CSV"""
        if not self.sweep_results:
            return
        
        # Create summary CSV
        csv_path = self.sweep_output_dir / "sweep_summary.csv"
        
        with open(csv_path, 'w') as f:
            # Header
            f.write("config_num,ee_roll_deg,ee_pitch_deg,drb_rot_roll,drb_rot_pitch,drb_rot_yaw,handedness,workspace_volume\n")
            
            # Data rows
            for idx, result in enumerate(self.sweep_results, 1):
                hand_str = "right" if result['handedness'] else "left"
                f.write(f"{idx},"
                       f"{result['ee_roll']},"
                       f"{result['ee_pitch']},"
                       f"{result['drb_rotation'][0]},"
                       f"{result['drb_rotation'][1]},"
                       f"{result['drb_rotation'][2]},"
                       f"{hand_str},"
                       f"{result['count']}\n")
        
        # Also create a human-readable text summary
        txt_path = self.sweep_output_dir / "sweep_summary.txt"
        
        with open(txt_path, 'w') as f:
            f.write("=" * 80 + "\n")
            f.write("HYPERPARAMETER SWEEP SUMMARY\n")
            f.write("=" * 80 + "\n\n")
            
            f.write(f"Total configurations tested: {len(self.sweep_results)}\n")
            f.write(f"Trajectories: {len(self.trajectories)}\n\n")
            
            # Find best configuration
            best = max(self.sweep_results, key=lambda x: x['count'])
            f.write("BEST CONFIGURATION:\n")
            f.write(f"  EE Roll: {best['ee_roll']}°\n")
            f.write(f"  EE Pitch: {best['ee_pitch']}°\n")
            f.write(f"  DRB Rotation: {best['drb_rotation']}\n")
            f.write(f"  Handedness: {'Right' if best['handedness'] else 'Left'}\n")
            f.write(f"  Workspace Volume: {best['count']} positions\n\n")
            
            # Summary statistics
            volumes = [r['count'] for r in self.sweep_results]
            f.write("STATISTICS:\n")
            f.write(f"  Mean workspace volume: {np.mean(volumes):.1f}\n")
            f.write(f"  Median workspace volume: {np.median(volumes):.1f}\n")
            f.write(f"  Std dev: {np.std(volumes):.1f}\n")
            f.write(f"  Min: {np.min(volumes)}\n")
            f.write(f"  Max: {np.max(volumes)}\n\n")
            
            f.write("=" * 80 + "\n")
            f.write("DETAILED RESULTS (sorted by workspace volume)\n")
            f.write("=" * 80 + "\n\n")
            
            # Sort by workspace volume
            sorted_results = sorted(self.sweep_results, key=lambda x: x['count'], reverse=True)
            
            for idx, result in enumerate(sorted_results, 1):
                hand_str = "Right" if result['handedness'] else "Left"
                f.write(f"{idx:3d}. EE_roll={result['ee_roll']:+4.0f}°  "
                       f"EE_pitch={result['ee_pitch']:+4.0f}°  "
                       f"DRB_rot={result['drb_rotation']}  "
                       f"{hand_str:5s}  "
                       f"Volume={result['count']:5d}\n")
        
        print(f"Saved summary to {csv_path} and {txt_path}")
        self.create_improvement_analysis_plot()
    def create_improvement_analysis_plot(self):
        """Create plot showing workspace improvement vs baseline"""
        import matplotlib.pyplot as plt
        
        # Separate results by roll-sweep and pitch-sweep
        roll_results = [r for r in self.sweep_results if r['ee_pitch'] == 10.0]
        pitch_results = [r for r in self.sweep_results if r['ee_roll'] == 0.0]
        
        # Find baseline
        baseline = next((r for r in self.sweep_results 
                        if r['ee_roll'] == 0.0 and r['ee_pitch'] == 10.0), None)
        
        if not baseline:
            return
        
        baseline_volume = baseline['count']
        
        # Create figure with 2 subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Plot 1: Roll sweep
        roll_angles = sorted(set(r['ee_roll'] for r in roll_results))
        roll_volumes = []
        for angle in roll_angles:
            configs = [r for r in roll_results if r['ee_roll'] == angle]
            avg_volume = np.mean([r['count'] for r in configs])
            roll_volumes.append(avg_volume)
        
        ax1.plot(roll_angles, roll_volumes, 'o-', linewidth=2, markersize=8)
        ax1.axhline(baseline_volume, color='red', linestyle='--', 
                   label=f'Baseline (roll=0°, pitch=10°): {baseline_volume} positions')
        ax1.set_xlabel('EE Roll (degrees)', fontsize=12)
        ax1.set_ylabel('Average Workspace Volume', fontsize=12)
        ax1.set_title('Workspace vs. EE Roll\n(pitch fixed at 10°)', fontsize=14)
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot 2: Pitch sweep
        pitch_angles = sorted(set(r['ee_pitch'] for r in pitch_results))
        pitch_volumes = []
        for angle in pitch_angles:
            configs = [r for r in pitch_results if r['ee_pitch'] == angle]
            avg_volume = np.mean([r['count'] for r in configs])
            pitch_volumes.append(avg_volume)
        
        ax2.plot(pitch_angles, pitch_volumes, 'o-', linewidth=2, markersize=8)
        ax2.axhline(baseline_volume, color='red', linestyle='--',
                   label=f'Baseline (roll=0°, pitch=10°): {baseline_volume} positions')
        ax2.set_xlabel('EE Pitch (degrees)', fontsize=12)
        ax2.set_ylabel('Average Workspace Volume', fontsize=12)
        ax2.set_title('Workspace vs. EE Pitch\n(roll fixed at 0°)', fontsize=14)
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        plt.tight_layout()
        
        # Save
        plot_path = self.sweep_output_dir / "improvement_analysis.png"
        plt.savefig(plot_path, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"Saved improvement analysis to {plot_path}")
        
        # Calculate improvement percentages
        max_roll_improvement = (max(roll_volumes) - baseline_volume) / baseline_volume * 100
        max_pitch_improvement = (max(pitch_volumes) - baseline_volume) / baseline_volume * 100
        
        # Add to text summary
        txt_path = self.sweep_output_dir / "sweep_summary.txt"
        with open(txt_path, 'a') as f:
            f.write("\n" + "=" * 80 + "\n")
            f.write("IMPROVEMENT OVER BASELINE\n")
            f.write("=" * 80 + "\n\n")
            f.write(f"Baseline configuration (roll=0°, pitch=10°): {baseline_volume} positions\n\n")
            f.write(f"Maximum improvement with roll articulation: {max_roll_improvement:+.1f}%\n")
            f.write(f"Maximum improvement with pitch articulation: {max_pitch_improvement:+.1f}%\n\n")
            
            if max_roll_improvement > 20 or max_pitch_improvement > 20:
                f.write("✅ CONCLUSION: Variable EE provides significant workspace improvement\n")
            else:
                f.write("⚠️  CONCLUSION: Variable EE provides minimal workspace improvement\n")
    
    def pause_hyperparam_sweep(self):
        """Pause the hyperparameter sweep"""
        if self.sweep_paused:
            self.sweep_paused = False
            self.pause_sweep_btn.setText("Pause")
            self.sweep_status_label.setText("Resuming...")
        else:
            self.sweep_paused = True
            self.pause_sweep_btn.setText("Resume")
            self.sweep_status_label.setText("Paused")
    
    def cancel_hyperparam_sweep(self):
        """Cancel the hyperparameter sweep"""
        reply = QMessageBox.question(self, "Cancel Sweep",
                                     "Are you sure you want to cancel the sweep?",
                                     QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        
        if reply == QMessageBox.StandardButton.Yes:
            self.sweep_cancelled = True
            self.sweep_status_label.setText("Cancelling...")

    # DRB Position Event Handlers
    def on_drb_pos_changed(self, axis, value):
        """Handle DRB position slider change"""
        idx = ['X', 'Y', 'Z'].index(axis)
        self.drb_position[idx] = float(value)
        self.drb_pos_controls[axis]['text'].setText(f"{value:.1f}")
        self.update_3d_plot()
    
    def on_drb_pos_text_changed(self, axis, text_widget):
        """Handle DRB position text input change"""
        try:
            value = float(text_widget.text())
            idx = ['X', 'Y', 'Z'].index(axis)
            
            # Clamp to valid range
            min_val, max_val = [(-1000, 1000), (-1000, 1500), (-200, 500)][idx]
            value = max(min_val, min(max_val, value))
            
            self.drb_position[idx] = value
            self.drb_pos_controls[axis]['slider'].setValue(int(value))
            text_widget.setText(f"{value:.1f}")
            self.update_3d_plot()
        except ValueError:
            # Reset to current value if invalid input
            idx = ['X', 'Y', 'Z'].index(axis)
            text_widget.setText(f"{self.drb_position[idx]:.1f}")
    
    # DRB Rotation Event Handlers
    def on_drb_rot_changed(self, axis, value):
        """Handle DRB rotation slider change"""
        idx = ['Roll', 'Pitch', 'Yaw'].index(axis)
        self.drb_rotation[idx] = float(value)
        self.drb_rot_controls[axis]['text'].setText(f"{value:.1f}")
        self.update_3d_plot()
    
    def on_drb_rot_text_changed(self, axis, text_widget):
        """Handle DRB rotation text input change"""
        try:
            value = float(text_widget.text())
            value = max(-180, min(180, value))  # Clamp to ±180
            
            idx = ['Roll', 'Pitch', 'Yaw'].index(axis)
            self.drb_rotation[idx] = value
            self.drb_rot_controls[axis]['slider'].setValue(int(value))
            text_widget.setText(f"{value:.1f}")
            self.update_3d_plot()
        except ValueError:
            idx = ['Roll', 'Pitch', 'Yaw'].index(axis)
            text_widget.setText(f"{self.drb_rotation[idx]:.1f}")
    
    # EE Orientation Event Handlers
    def on_ee_pitch_changed(self, value):
        """Handle EE pitch slider change"""
        self.ee_pitch = float(value)
        self.ee_pitch_text.setText(f"{value:.1f}")
    
    def on_ee_pitch_text_changed(self):
        """Handle EE pitch text input change"""
        try:
            value = float(self.ee_pitch_text.text())
            value = max(-180, min(180, value))
            self.ee_pitch = value
            self.ee_pitch_slider.setValue(int(value))
            self.ee_pitch_text.setText(f"{value:.1f}")
        except ValueError:
            self.ee_pitch_text.setText(f"{self.ee_pitch:.1f}")
    
    def on_ee_roll_changed(self, value):
        """Handle EE roll slider change"""
        self.ee_roll = float(value)
        self.ee_roll_text.setText(f"{value:.1f}")
    
    def on_ee_roll_text_changed(self):
        """Handle EE roll text input change"""
        try:
            value = float(self.ee_roll_text.text())
            value = max(-180, min(180, value))
            self.ee_roll = value
            self.ee_roll_slider.setValue(int(value))
            self.ee_roll_text.setText(f"{value:.1f}")
        except ValueError:
            self.ee_roll_text.setText(f"{self.ee_roll:.1f}")

    # Trajectory Management Methods
    def add_trajectory(self):
        """Add a new trajectory"""
        # Generate default name
        name = f"Trajectory {len(self.trajectories) + 1}"
        
        # Default start and end points in DRB frame
        start_drb = [0.0, 0.0, 50.0]
        end_drb = [0.0, 0.0, 50.0]
        
        traj = Trajectory(name, start_drb, end_drb)
        self.trajectories.append(traj)
        
        # Update list widget
        self.trajectory_list.addItem(name)
        
        # Select the new trajectory
        self.trajectory_list.setCurrentRow(len(self.trajectories) - 1)
        
        self.update_3d_plot()
        self.statusBar.showMessage(f"Added {name}")
    
    def remove_trajectory(self):
        """Remove selected trajectory"""
        if self.selected_trajectory_idx is None:
            return
        
        traj = self.trajectories[self.selected_trajectory_idx]
        self.trajectories.pop(self.selected_trajectory_idx)
        self.trajectory_list.takeItem(self.selected_trajectory_idx)
        
        self.selected_trajectory_idx = None
        self.solve_ik_btn.setEnabled(False)
        
        self.update_3d_plot()
        self.statusBar.showMessage(f"Removed {traj.name}")
    
    def rename_trajectory(self):
        """Rename selected trajectory"""
        if self.selected_trajectory_idx is None:
            return
        
        traj = self.trajectories[self.selected_trajectory_idx]
        
        new_name, ok = QInputDialog.getText(
            self,
            "Rename Trajectory",
            "Enter new name:",
            QLineEdit.EchoMode.Normal,
            traj.name
        )
        
        if ok and new_name:
            traj.name = new_name
            self.trajectory_list.item(self.selected_trajectory_idx).setText(new_name)
            self.statusBar.showMessage(f"Renamed to {new_name}")
    
    def on_trajectory_selected(self):
        """Handle trajectory selection"""
        selected_items = self.trajectory_list.selectedItems()
        if selected_items:
            self.selected_trajectory_idx = self.trajectory_list.row(selected_items[0])
            self.solve_ik_btn.setEnabled(True)
            self.update_3d_plot()
        else:
            self.selected_trajectory_idx = None
            self.solve_ik_btn.setEnabled(False)

        if self.selected_trajectory_idx is not None:
            traj = self.trajectories[self.selected_trajectory_idx]
            self.update_trajectory_text_boxes(traj)
        
        self.update_3d_plot()
    
    def adjust_point(self, axis, delta):
        """Adjust trajectory point by delta mm"""
        if self.selected_trajectory_idx is None:
            return
        
        traj = self.trajectories[self.selected_trajectory_idx]
        axis_idx = ['X', 'Y', 'Z'].index(axis)
        
        # Determine which point to edit
        if self.edit_start_radio.isChecked():
            traj.start_drb[axis_idx] += delta
            point_name = "start"
        else:
            traj.end_drb[axis_idx] += delta
            point_name = "end"
        
        self.update_3d_plot()
        self.statusBar.showMessage(
            f"Adjusted {traj.name} {point_name} {axis} by {delta:+.1f}mm"
        )

    # File Operations
    def save_configuration(self):
        """Save DRB position and trajectories to JSON file"""
        script_dir = str(Path(__file__).parent)
        
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Configuration",
            script_dir,
            "JSON Files (*.json);;All Files (*)"
        )
        
        if not file_path:
            return
        
        config = {
            'drb_position': self.drb_position.tolist(),
            'drb_rotation': self.drb_rotation.tolist(),
            'trajectories': [traj.to_dict() for traj in self.trajectories]
        }
        
        try:
            with open(file_path, 'w') as f:
                json.dump(config, f, indent=2)
            self.statusBar.showMessage(f"Saved to {file_path}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save:\n{str(e)}")
    
    def load_configuration(self):
        """Load DRB position and trajectories from JSON file"""
        script_dir = str(Path(__file__).parent)
        
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Configuration",
            script_dir,
            "JSON Files (*.json);;All Files (*)"
        )
        
        if not file_path:
            return
        
        try:
            with open(file_path, 'r') as f:
                config = json.load(f)
            
            # Load DRB position
            self.drb_position = np.array(config['drb_position'])
            for i, axis in enumerate(['X', 'Y', 'Z']):
                value = self.drb_position[i]
                self.drb_pos_controls[axis]['slider'].setValue(int(value))
                self.drb_pos_controls[axis]['text'].setText(f"{value:.1f}")
            
            # Load DRB rotation
            if 'drb_rotation' in config:
                self.drb_rotation = np.array(config['drb_rotation'])
                for i, axis in enumerate(['Roll', 'Pitch', 'Yaw']):
                    value = self.drb_rotation[i]
                    self.drb_rot_controls[axis]['slider'].setValue(int(value))
                    self.drb_rot_controls[axis]['text'].setText(f"{value:.1f}")
            
            # Load trajectories
            self.trajectories = [Trajectory.from_dict(d) for d in config['trajectories']]
            self.trajectory_list.clear()
            for traj in self.trajectories:
                self.trajectory_list.addItem(traj.name)
            
            self.update_3d_plot()
            self.statusBar.showMessage(f"Loaded from {file_path}")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load:\n{str(e)}")
    
    # Helper Methods
    def get_drb_transform(self):
        """Get DRB frame transform (world frame to DRB frame)"""
        # Create rotation matrix from Euler angles (intrinsic ZYX)
        roll_rad = np.deg2rad(self.drb_rotation[0])
        pitch_rad = np.deg2rad(self.drb_rotation[1])
        yaw_rad = np.deg2rad(self.drb_rotation[2])
        
        # Rotation matrices
        Rz = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                       [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                       [0, 0, 1]])
        
        Ry = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                       [0, 1, 0],
                       [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
        
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll_rad), -np.sin(roll_rad)],
                       [0, np.sin(roll_rad), np.cos(roll_rad)]])
        
        # Combined rotation (intrinsic ZYX: R = Rz * Ry * Rx)
        R = Rz @ Ry @ Rx
        
        # Create 4x4 homogeneous transform
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = self.drb_position
        
        return T
    
    def transform_point_drb_to_world(self, point_drb):
        """Transform point from DRB frame to world frame"""
        T = self.get_drb_transform()
        point_homog = np.append(point_drb, 1.0)
        point_world = T @ point_homog
        return point_world[:3]
    
    def update_joint_display(self):
        """Update joint angle display"""
        if self.current_joints is None:
            for label in self.joint_labels.values():
                label.setText("N/A")
            return
        
        try:
            self.joint_labels['Vertical'].setText(f"{self.current_joints.vertical:.2f} mm")
            self.joint_labels['Shoulder'].setText(f"{self.current_joints.shoulder.degrees():.2f}°")
            self.joint_labels['Elbow'].setText(f"{self.current_joints.elbow.degrees():.2f}°")
            self.joint_labels['Roll'].setText(f"{self.current_joints.roll.degrees():.2f}°")
            self.joint_labels['Pitch'].setText(f"{self.current_joints.pitch.degrees():.2f}°")
        except:
            for label in self.joint_labels.values():
                label.setText("Error")

    # IK Solving Methods
    # IK Solving Methods
    def solve_ik_for_trajectory(self):
        """Solve IK for all points in the selected trajectory"""
        if self.selected_trajectory_idx is None:
            return
        
        # Stop animation if running
        if self.is_playing:
            self.toggle_play_pause()
        
        traj = self.trajectories[self.selected_trajectory_idx]
        
        # Get interpolated points along trajectory (20mm spacing)
        points_drb = traj.interpolate_points(spacing_mm=20.0)
        
        # Compute robotTool_X_eeTip transform
        robot_tool_x_ee_tip = compute_robot_tool_to_ee_tip(self.ee_roll, self.ee_pitch)
        
        # Track solve time
        start_time = time.time()
        
        all_solutions = []
        failed_points = []
        
        # Solve IK for each point
        for i, point_drb in enumerate(points_drb):
            # Transform point to world frame
            point_world = self.transform_point_drb_to_world(point_drb)
            
            # Get trajectory direction in world frame
            direction_drb = traj.get_direction()
            # Transform direction (rotation only, no translation)
            T_drb = self.get_drb_transform()
            direction_world = T_drb[:3, :3] @ direction_drb
            direction_world = direction_world / np.linalg.norm(direction_world)
            
            # Create target pose (point + direction)
            target_point = gk.Point3d(point_world[0], point_world[1], point_world[2])
            
            # Create UnitVector3d by first creating Vector3d, then calling unitVector()
            direction_vector = gk.Vector3d(direction_world[0], direction_world[1], direction_world[2])
            target_direction = direction_vector.unitVector()
            
            target_ray = gk.Ray3d(target_point, target_direction)
            
            # Solve IK
            try:
                # Use last successful solution as current joints for seeding
                current = self.last_successful_joints if self.last_successful_joints else self.current_joints
                
                # Use findSolutionQuick
                success, solution = self.ik_solver.findSolutionQuick(
                    target_ray,
                    robot_tool_x_ee_tip,
                    current,
                    self.validator,
                    wristMode=False,
                    rightHand=True,
                    tolerance=0.01
                )
                
                # Check if solution is valid
                if success:
                    all_solutions.append(solution)
                    self.last_successful_joints = solution
                else:
                    failed_points.append(i)
                    self.statusBar.showMessage(f"IK solution not found for point {i+1}/{len(points_drb)}")
                    
            except Exception as e:
                failed_points.append(i)
                self.statusBar.showMessage(f"IK solve failed for point {i+1}/{len(points_drb)}: {str(e)}")
        
        solve_time = time.time() - start_time
        self.solve_time_label.setText(f"{solve_time*1000:.1f} ms")
        
        # Store solutions and enable animation controls
        if all_solutions:
            self.trajectory_solutions = all_solutions
            self.current_solution_idx = 0
            
            # Enable animation controls
            self.play_pause_btn.setEnabled(True)
            self.prev_solution_btn.setEnabled(True)
            self.next_solution_btn.setEnabled(True)
            
            # Update display with first solution
            self.current_joints = self.trajectory_solutions[0]
            self.update_joint_display()
            self.update_solution_label()
            self.update_3d_plot()
            
            if failed_points:
                msg = f"Solved {len(all_solutions)}/{len(points_drb)} points. Failed: {failed_points}"
                self.statusBar.showMessage(msg)
                QMessageBox.warning(self, "Partial Success", msg)
            else:
                self.statusBar.showMessage(f"All {len(points_drb)} points reachable! Use Play to animate.")
        else:
            self.trajectory_solutions = []
            self.play_pause_btn.setEnabled(False)
            self.prev_solution_btn.setEnabled(False)
            self.next_solution_btn.setEnabled(False)
            self.statusBar.showMessage("No reachable points found")
            QMessageBox.critical(self, "Error", "Trajectory is unreachable")
    
    def toggle_play_pause(self):
        """Toggle animation play/pause"""
        if not self.trajectory_solutions:
            return
        
        self.is_playing = not self.is_playing
        
        if self.is_playing:
            self.play_pause_btn.setText("Pause")
            self.animation_timer.start(self.animation_speed)
        else:
            self.play_pause_btn.setText("Play")
            self.animation_timer.stop()
    
    def animation_step(self):
        """Advance to next solution in animation"""
        if not self.trajectory_solutions:
            return
        
        self.next_trajectory_solution()
    
    def next_trajectory_solution(self):
        """Show next joint solution along trajectory"""
        if not self.trajectory_solutions:
            return
        
        self.current_solution_idx = (self.current_solution_idx + 1) % len(self.trajectory_solutions)
        self.current_joints = self.trajectory_solutions[self.current_solution_idx]
        self.update_joint_display()
        self.update_solution_label()
        self.update_3d_plot()
    
    def prev_trajectory_solution(self):
        """Show previous joint solution along trajectory"""
        if not self.trajectory_solutions:
            return
        
        self.current_solution_idx = (self.current_solution_idx - 1) % len(self.trajectory_solutions)
        self.current_joints = self.trajectory_solutions[self.current_solution_idx]
        self.update_joint_display()
        self.update_solution_label()
        self.update_3d_plot()
    
    def update_solution_label(self):
        """Update the solution index label"""
        if self.trajectory_solutions:
            self.solution_idx_label.setText(f"Solution: {self.current_solution_idx + 1}/{len(self.trajectory_solutions)}")
        else:
            self.solution_idx_label.setText("Solution: N/A")
    
    def on_speed_changed(self, value):
        """Handle animation speed change"""
        self.animation_speed = value
        self.speed_label.setText(f"{value}ms")
        if self.is_playing:
            self.animation_timer.setInterval(value)

    # Reachability Analysis Methods
    def analyze_reachability(self):
        """Analyze reachability for all trajectories across DRB position grid"""
        if not self.trajectories:
            QMessageBox.warning(self, "Warning", "No trajectories defined")
            return
        
        self.reachability_running = True
        self.reachability_cancelled = False
        
        # Show progress UI
        self.reach_progress.setVisible(True)
        self.cancel_reach_btn.setVisible(True)
        self.analyze_reach_btn.setEnabled(False)
        
        # Define grid (50mm resolution)
        x_range = np.arange(-1000, 1001, 50)
        y_range = np.arange(-1000, 1501, 50)
        z_range = np.arange(-200, 501, 50)
        
        total_positions = len(x_range) * len(y_range) * len(z_range)
        
        # Sort trajectories by most horizontal (smallest z component)
        sorted_trajs = sorted(self.trajectories, key=lambda t: abs(t.get_direction()[2]))
        
        # Start with all grid positions
        valid_positions = []
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    valid_positions.append([x, y, z])
        
        self.reach_progress.setMaximum(len(sorted_trajs) * len(valid_positions))
        progress_count = 0
        last_update_time = time.time()
        
        # Filter progressively through each trajectory
        for traj_idx, traj in enumerate(sorted_trajs):
            if self.reachability_cancelled:
                break
            
            newly_valid = []
            robot_tool_x_ee_tip = compute_robot_tool_to_ee_tip(self.ee_roll, self.ee_pitch)
            
            # Get interpolated points for this trajectory
            points_drb = traj.interpolate_points(spacing_mm=20.0)
            
            # Use initial joints for first trajectory, last successful for rest
            if traj_idx == 0:
                current_joints = gk.Joints(
                    200.0,
                    gk.Angle.fromDegrees(0.0),
                    gk.Angle.fromDegrees(0.00001),
                    gk.Angle.fromDegrees(0.00001),
                    gk.Angle.fromDegrees(0.0)
                )
            else:
                current_joints = self.last_successful_joints if self.last_successful_joints else self.current_joints
            
            for drb_pos in valid_positions:
                if self.reachability_cancelled:
                    break
                
                # Test if all points in trajectory are reachable from this DRB position
                all_reachable = True
                
                # Save current DRB state
                saved_drb_pos = self.drb_position.copy()
                self.drb_position = np.array(drb_pos)
                
                for point_drb in points_drb:
                    # Transform to world frame
                    point_world = self.transform_point_drb_to_world(point_drb)
                    direction_drb = traj.get_direction()
                    T_drb = self.get_drb_transform()
                    direction_world = T_drb[:3, :3] @ direction_drb
                    direction_world = direction_world / np.linalg.norm(direction_world)
                    
                    # Create target
                    target_point = gk.Point3d(point_world[0], point_world[1], point_world[2])
                    target_direction = gk.Vector3d(direction_world[0], direction_world[1], direction_world[2]).unitVector()
                    
                    target_ray = gk.Ray3d(target_point, target_direction)
                    
                    # Solve IK
                    success, solution = self.ik_solver.findSolutionQuick(
                        target_ray,
                        robot_tool_x_ee_tip,
                        current_joints,
                        self.validator,
                        wristMode=False,
                        rightHand=current_joints.elbow.degrees() >= 0.0,
                        tolerance=0.01
                    )
                        
                    if not success:
                        all_reachable = False
                        break
                    current_joints = solution  # Update for next point
                
                # Restore DRB state
                self.drb_position = saved_drb_pos
                
                if all_reachable:
                    newly_valid.append(drb_pos)
                
                # Update progress
                progress_count += 1
                current_time = time.time()
                if current_time - last_update_time >= 1.0:  # Update every second
                    self.reach_progress.setValue(progress_count)
                    percent = (progress_count / self.reach_progress.maximum()) * 100
                    self.statusBar.showMessage(f"Analyzing: {percent:.1f}% complete")
                    QApplication.processEvents()
                    last_update_time = current_time
            
            # Filter down to newly valid positions
            valid_positions = newly_valid
            
            if not valid_positions:
                break  # No valid positions left
        
        # Hide progress UI
        self.reach_progress.setVisible(False)
        self.cancel_reach_btn.setVisible(False)
        self.analyze_reach_btn.setEnabled(True)
        self.reachability_running = False
        
        if self.reachability_cancelled:
            self.statusBar.showMessage("Reachability analysis cancelled")
            return
        
        # Store results and visualize
        self.reachability_results = valid_positions
        
        if valid_positions:
            msg = f"Found {len(valid_positions)} valid DRB positions"
            self.statusBar.showMessage(msg)
            QMessageBox.information(self, "Success", msg)
            self.update_3d_plot()  # Will show point cloud
        else:
            self.statusBar.showMessage("No valid DRB positions found")
            QMessageBox.warning(self, "No Solutions", "No DRB positions can reach all trajectories")
    
    def cancel_reachability(self):
        """Cancel ongoing reachability analysis"""
        self.reachability_cancelled = True
        self.statusBar.showMessage("Cancelling reachability analysis...")

    # 3D Visualization Methods
    # 3D Visualization Methods
    def update_3d_plot(self):
        """Update the 3D visualization"""
        if self.ax is None:
            return
        
        # Save current view AND axis limits
        try:
            self.prev_view["elev"] = self.ax.elev
            self.prev_view["azim"] = self.ax.azim
            self.prev_view["roll"] = self.ax.roll
            self.prev_view["xlim"] = self.ax.get_xlim()
            self.prev_view["ylim"] = self.ax.get_ylim()
            self.prev_view["zlim"] = self.ax.get_zlim()
        except:
            pass
        
        # Clear and recreate
        self.ax.clear()
        
        # Set labels and title
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        
        # Draw world origin
        self.ax.scatter([0], [0], [0], c='black', marker='o', s=100, label='World Origin')
        
        # Draw DRB frame
        self.draw_drb_frame()
        
        # Draw trajectories
        self.draw_trajectories()
        
        # Draw robot arm
        self.draw_robot_arm()
        
        # Draw reachability results if available
        if hasattr(self, 'reachability_results') and self.reachability_results:
            self.draw_reachability_cloud()
        
        # Restore axis limits (zoom) if they were saved, otherwise use defaults
        if "xlim" in self.prev_view:
            self.ax.set_xlim(self.prev_view["xlim"])
            self.ax.set_ylim(self.prev_view["ylim"])
            self.ax.set_zlim(self.prev_view["zlim"])
        else:
            self.ax.set_xlim([-1000, 1000])
            self.ax.set_ylim([-1000, 1500])
            self.ax.set_zlim([-400, 1000])
        
        # Set equal aspect ratio
        self.ax.set_box_aspect([1, 1, 1])
        
        # Restore view angles
        try:
            self.ax.view_init(elev=self.prev_view["elev"], azim=self.prev_view["azim"], roll=self.prev_view["roll"])
        except:
            self.ax.view_init(elev=30, azim=30)
        
        self.canvas.draw()
    
    def draw_drb_frame(self):
        """Draw DRB coordinate frame"""
        T = self.get_drb_transform()
        origin = T[:3, 3]
        
        # Draw coordinate axes (100mm long)
        axis_length = 100.0
        x_axis = T[:3, :3] @ np.array([axis_length, 0, 0])
        y_axis = T[:3, :3] @ np.array([0, axis_length, 0])
        z_axis = T[:3, :3] @ np.array([0, 0, axis_length])
        
        self.ax.quiver(origin[0], origin[1], origin[2],
                    x_axis[0], x_axis[1], x_axis[2],
                    color='red', arrow_length_ratio=0.2, linewidth=2, label='DRB X')
        self.ax.quiver(origin[0], origin[1], origin[2],
                    y_axis[0], y_axis[1], y_axis[2],
                    color='green', arrow_length_ratio=0.2, linewidth=2, label='DRB Y')
        self.ax.quiver(origin[0], origin[1], origin[2],
                    z_axis[0], z_axis[1], z_axis[2],
                    color='blue', arrow_length_ratio=0.2, linewidth=2, label='DRB Z')
        
        # # Draw stick figure with DRB origin at belly/waist
        # # All measurements relative to DRB origin (which is now at the waist)
        
        # # Upper body (going UP from waist in +Y direction)
        # waist = origin  # DRB origin IS the waist
        # chest = origin + T[:3, :3] @ np.array([0, 240, 0])       # 240mm up to chest
        # neck = origin + T[:3, :3] @ np.array([0, 500, 0])        # 500mm up to neck
        # head_center = origin + T[:3, :3] @ np.array([0, 620, 0]) # 620mm up to head center
        
        # # Lower body (going DOWN from waist in -Y direction)
        # pelvis = origin + T[:3, :3] @ np.array([0, -160, 0])     # 160mm down to pelvis
        # feet = origin + T[:3, :3] @ np.array([0, -1060, 0])      # 900mm legs + 160mm hip = 1060mm total
        
        # # Head (circle) - diameter ~200mm
        # from matplotlib.patches import Circle
        # from mpl_toolkits.mplot3d import art3d
        # head_radius = 100  # 200mm diameter head
        # circle = Circle((head_center[0], head_center[1]), head_radius, color='black', fill=False, linewidth=2)
        # self.ax.add_patch(circle)
        # art3d.pathpatch_2d_to_3d(circle, z=head_center[2], zdir='z')
        
        # # Spine
        # self.ax.plot([neck[0], chest[0]], [neck[1], chest[1]], [neck[2], chest[2]], 
        #             'k-', linewidth=3)
        # self.ax.plot([chest[0], waist[0]], [chest[1], waist[1]], [chest[2], waist[2]], 
        #             'k-', linewidth=3)
        # self.ax.plot([waist[0], pelvis[0]], [waist[1], pelvis[1]], [waist[2], pelvis[2]], 
        #             'k-', linewidth=3)
        
        # # Arms (from chest, extending sideways in X)
        # left_shoulder = chest + T[:3, :3] @ np.array([-200, 0, 0])   # Shoulder width
        # right_shoulder = chest + T[:3, :3] @ np.array([200, 0, 0])
        # left_elbow = chest + T[:3, :3] @ np.array([-250, -300, 0])   # Upper arm ~300mm down
        # right_elbow = chest + T[:3, :3] @ np.array([250, -300, 0])
        # left_hand = chest + T[:3, :3] @ np.array([-250, -650, 0])    # Forearm ~350mm more
        # right_hand = chest + T[:3, :3] @ np.array([250, -650, 0])
        
        # # Upper arms
        # self.ax.plot([left_shoulder[0], left_elbow[0]], 
        #             [left_shoulder[1], left_elbow[1]], 
        #             [left_shoulder[2], left_elbow[2]], 'k-', linewidth=2)
        # self.ax.plot([right_shoulder[0], right_elbow[0]], 
        #             [right_shoulder[1], right_elbow[1]], 
        #             [right_shoulder[2], right_elbow[2]], 'k-', linewidth=2)
        
        # # Forearms
        # self.ax.plot([left_elbow[0], left_hand[0]], 
        #             [left_elbow[1], left_hand[1]], 
        #             [left_elbow[2], left_hand[2]], 'k-', linewidth=2)
        # self.ax.plot([right_elbow[0], right_hand[0]], 
        #             [right_elbow[1], right_hand[1]], 
        #             [right_elbow[2], right_hand[2]], 'k-', linewidth=2)
        
        # # Legs (from pelvis to feet)
        # left_hip = pelvis + T[:3, :3] @ np.array([-100, 0, 0])      # Hip width
        # right_hip = pelvis + T[:3, :3] @ np.array([100, 0, 0])
        # left_knee = pelvis + T[:3, :3] @ np.array([-100, -450, 0])  # Upper leg ~450mm
        # right_knee = pelvis + T[:3, :3] @ np.array([100, -450, 0])
        # left_foot = feet + T[:3, :3] @ np.array([-100, 0, 0])       # Lower leg ~450mm
        # right_foot = feet + T[:3, :3] @ np.array([100, 0, 0])
        
        # # Thighs
        # self.ax.plot([left_hip[0], left_knee[0]], 
        #             [left_hip[1], left_knee[1]], 
        #             [left_hip[2], left_knee[2]], 'k-', linewidth=3)
        # self.ax.plot([right_hip[0], right_knee[0]], 
        #             [right_hip[1], right_knee[1]], 
        #             [right_hip[2], right_knee[2]], 'k-', linewidth=3)
        
        # # Shins
        # self.ax.plot([left_knee[0], left_foot[0]], 
        #             [left_knee[1], left_foot[1]], 
        #             [left_knee[2], left_foot[2]], 'k-', linewidth=3)
        # self.ax.plot([right_knee[0], right_foot[0]], 
        #             [right_knee[1], right_foot[1]], 
        #             [right_knee[2], right_foot[2]], 'k-', linewidth=3)
    
    def draw_trajectories(self):
        """Draw all trajectories"""
        for idx, traj in enumerate(self.trajectories):
            # Transform points to world frame
            start_world = self.transform_point_drb_to_world(traj.start_drb)
            end_world = self.transform_point_drb_to_world(traj.end_drb)
            
            # Choose color (highlight selected)
            if idx == self.selected_trajectory_idx:
                color = 'orange'
                linewidth = 3
            else:
                color = 'black'
                linewidth = 2
            
            # Draw trajectory direction arrow (end -> start)
            direction = traj.get_direction()
            T_drb = self.get_drb_transform()
            direction_world = T_drb[:3, :3] @ direction
            
            # Calculate arrow length (full trajectory length)
            arrow_length = np.linalg.norm(start_world - end_world)
            direction_world = direction_world / np.linalg.norm(direction_world) * arrow_length
            
            # Draw arrow from end point in direction of start point
            self.ax.quiver(end_world[0], end_world[1], end_world[2],
                          direction_world[0], direction_world[1], direction_world[2],
                          color=color, arrow_length_ratio=0.1, linewidth=linewidth,
                          label=traj.name if idx == self.selected_trajectory_idx else '')
    
    def draw_robot_arm(self):
        """Draw robot arm using forward kinematics"""
        if self.current_joints is None:
            return
        
        try:
            # Get base offset
            base_offset_point = self.fk.getBaseOffsets()
            base_pos = np.array([base_offset_point.x(), base_offset_point.y(), base_offset_point.z()])

            # Draw robot base as a cube [800, 300, 500] mm centered at vertical column origin
            self.draw_robot_base_cube(base_pos)
            
            # Compute link positions using FK chain
            # Base -> Vertical
            vertical_T = self.fk.base_T_vertical(self.current_joints.vertical)
            vertical_pos = self.transform_affine_origin(vertical_T)
            
            # Vertical -> Shoulder
            shoulder_T = self.fk.vertical_T_shoulder(self.current_joints.shoulder)
            shoulder_pos = self.transform_affine_origin(vertical_T * shoulder_T)
            
            # Shoulder -> Elbow
            elbow_T = self.fk.shoulder_T_elbow(self.current_joints.elbow)
            elbow_pos = self.transform_affine_origin(vertical_T * shoulder_T * elbow_T)
            
            # Elbow -> Roll
            roll_T = self.fk.elbow_T_roll(self.current_joints.roll)
            roll_pos = self.transform_affine_origin(vertical_T * shoulder_T * elbow_T * roll_T)
            
            # Roll -> Pitch
            pitch_T = self.fk.roll_T_pitch(self.current_joints.pitch)
            pitch_pos = self.transform_affine_origin(vertical_T * shoulder_T * elbow_T * roll_T * pitch_T)
            
            # Get tool position
            robot_tool_x_ee_tip = compute_robot_tool_to_ee_tip(self.ee_roll, self.ee_pitch)
            ee_ray = self.fk.eeTip_base(self.current_joints, robot_tool_x_ee_tip)
            ee_point = ee_ray.point()
            tool_pos = np.array([ee_point.x(), ee_point.y(), ee_point.z()])
            
            # Draw links
            positions = [base_pos, vertical_pos, shoulder_pos, elbow_pos, roll_pos, pitch_pos, tool_pos]
            
            for i in range(len(positions) - 1):
                self.ax.plot([positions[i][0], positions[i+1][0]],
                            [positions[i][1], positions[i+1][1]],
                            [positions[i][2], positions[i+1][2]],
                            'b-', linewidth=4, alpha=0.8)
            
            # Draw joints
            for pos in positions:
                self.ax.scatter(*pos, c='darkblue', marker='o', s=100)
            
            # Draw EE orientation arrow (along EE Z-axis, 100mm)
            ee_dir = ee_ray.dir()
            ee_direction_vec = np.array([ee_dir.x(), ee_dir.y(), ee_dir.z()]) * 100
            ee_direction_vec = -ee_direction_vec # show direction pointing out of the tip
            
            self.ax.quiver(tool_pos[0], tool_pos[1], tool_pos[2],
                          ee_direction_vec[0], ee_direction_vec[1], ee_direction_vec[2],
                          color='purple', arrow_length_ratio=0.2, linewidth=3,
                          label='EE Orientation')
            
        except Exception as e:
            print(f"Error drawing robot arm: {e}")

    def draw_robot_base_cube(self, center, ax=None):
        """Draw robot base as a cube centered at the vertical column origin
        
        Args:
            center: Center position [x, y, z]
            ax: Matplotlib 3D axis (defaults to self.ax)
        """
        if ax is None:
            ax = self.ax
            
        # Cube dimensions [width_x, depth_y, height_z]
        width = 1000   # X dimension
        depth = 500   # Y dimension
        height = 1000  # Z dimension
        
        # Calculate corners (cube centered at 'center')
        x_min = center[0] - 3*width / 4
        x_max = center[0] + width / 4
        y_min = center[1] - depth / 2
        y_max = center[1] + depth / 2
        z_min = center[2] - height
        z_max = center[2]
        
        # Define the 8 vertices of the cube
        vertices = [
            [x_min, y_min, z_min],  # 0
            [x_max, y_min, z_min],  # 1
            [x_max, y_max, z_min],  # 2
            [x_min, y_max, z_min],  # 3
            [x_min, y_min, z_max],  # 4
            [x_max, y_min, z_max],  # 5
            [x_max, y_max, z_max],  # 6
            [x_min, y_max, z_max],  # 7
        ]
        
        # Define the 6 faces of the cube
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # Right
        ]
        
        # Create 3D polygon collection for the cube
        cube = Poly3DCollection(faces, alpha=0.3, facecolor='gray', edgecolor='black', linewidth=1)
        ax.add_collection3d(cube)  # ← USE THE PASSED ax PARAMETER
    
    def transform_affine_origin(self, affine):
        """Extract origin from Affine3d as numpy array"""
        origin = affine.origin()
        return np.array([origin.x(), origin.y(), origin.z()])
    
    def draw_reachability_cloud(self):
        """Draw reachability analysis results as point cloud"""
        if not self.reachability_results:
            return
        
        positions = np.array(self.reachability_results)
        self.ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
                       c='lime', marker='.', s=1, alpha=0.3,
                       label=f'Reachable ({len(positions)} points)')
        
    def on_trajectory_coord_changed(self):
        """Handle trajectory coordinate text box changes (Enter key pressed)"""
        # Just validate the input, actual update happens on Apply button
        sender = self.sender()
        try:
            value = float(sender.text())
            sender.setText(f"{value:.2f}")
        except ValueError:
            if self.selected_trajectory_idx is not None:
                traj = self.trajectories[self.selected_trajectory_idx]
                # Reset to current value
                self.update_trajectory_text_boxes(traj)
    
    def apply_trajectory_coordinates(self):
        """Apply manually entered trajectory coordinates"""
        if self.selected_trajectory_idx is None:
            QMessageBox.warning(self, "No Selection", "Please select a trajectory first.")
            return
        
        try:
            # Read coordinates from text boxes
            start_x = float(self.start_x_text.text())
            start_y = float(self.start_y_text.text())
            start_z = float(self.start_z_text.text())
            
            end_x = float(self.end_x_text.text())
            end_y = float(self.end_y_text.text())
            end_z = float(self.end_z_text.text())
            
            # Update trajectory
            traj = self.trajectories[self.selected_trajectory_idx]
            traj.start_drb = np.array([start_x, start_y, start_z])
            traj.end_drb = np.array([end_x, end_y, end_z])
            
            # Update visualization
            self.update_3d_plot()
            self.statusBar.showMessage(
                f"Updated trajectory '{traj.name}': "
                f"Start=[{start_x:.1f}, {start_y:.1f}, {start_z:.1f}], "
                f"End=[{end_x:.1f}, {end_y:.1f}, {end_z:.1f}]"
            )
            
        except ValueError as e:
            QMessageBox.warning(
                self, 
                "Invalid Input", 
                f"Please enter valid numbers for all coordinates.\n\nError: {str(e)}"
            )
    
    def update_trajectory_text_boxes(self, traj):
        """Update text boxes with trajectory coordinates"""
        self.start_x_text.setText(f"{traj.start_drb[0]:.2f}")
        self.start_y_text.setText(f"{traj.start_drb[1]:.2f}")
        self.start_z_text.setText(f"{traj.start_drb[2]:.2f}")
        
        self.end_x_text.setText(f"{traj.end_drb[0]:.2f}")
        self.end_y_text.setText(f"{traj.end_drb[1]:.2f}")
        self.end_z_text.setText(f"{traj.end_drb[2]:.2f}")

    def adjust_trajectory_point(self, point_type, axis, delta):
        """Adjust trajectory point coordinates"""
        if self.selected_trajectory_idx is None:
            return
        
        traj = self.trajectories[self.selected_trajectory_idx]
        axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        
        if point_type == 'start':
            traj.start_drb[axis_idx] += delta
        else:  # 'end'
            traj.end_drb[axis_idx] += delta
        
        # Update text boxes
        self.update_trajectory_text_boxes(traj)
        
        # Update visualization
        self.update_3d_plot()
        self.statusBar.showMessage(
            f"Adjusted {point_type} {axis.upper()} by {delta:+.1f}mm"
        )


# Main entry point
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = IKSimulatorApp()
    window.show()
    sys.exit(app.exec())