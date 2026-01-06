"""
Standalone Desktop App for Robot Workspace Visualization
Requires: PyQt6, matplotlib, numpy, pandas, pyqtgraph (optional for better 3D)
"""

import os
os.environ['QT_ENABLE_HIGHDPI_SCALING'] = '1'
os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '1'

import sys
import numpy as np
import pandas as pd
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QSlider, QCheckBox, QPushButton,
                             QFileDialog, QComboBox, QGroupBox, QStatusBar)
from PyQt6.QtCore import Qt
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

class WorkspaceVisualizerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.df = None
        self.filtered_df = None
        self.prev_view = {"elev": 30, "azim": 30, "roll": 0}
        self.ax = None
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle('Reachability visializer')
        self.setGeometry(100, 100, 1400, 900)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel for controls
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, stretch=1)
        
        # Right panel for 3D plot
        plot_panel = self.create_plot_panel()
        main_layout.addWidget(plot_panel, stretch=3)
        
        # Status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('Ready. Please load a CSV file.')
        
    def create_control_panel(self):
        """Create left control panel with sliders"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # File selection
        file_group = QGroupBox("Data File")
        file_layout = QVBoxLayout()
        
        self.load_button = QPushButton("Load CSV File")
        self.load_button.clicked.connect(self.load_csv)
        file_layout.addWidget(self.load_button)
        
        self.file_label = QLabel("No file loaded")
        self.file_label.setWordWrap(True)
        file_layout.addWidget(self.file_label)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # Parameter controls
        param_group = QGroupBox("Parameters")
        param_layout = QVBoxLayout()
        
        # EE Roll
        param_layout.addWidget(QLabel("EE Roll deg:\nStandard EE roll is 0"))
        self.ee_roll_combo = QComboBox()
        self.ee_roll_combo.currentIndexChanged.connect(self.update_plot)
        param_layout.addWidget(self.ee_roll_combo)
        
        # EE Pitch
        param_layout.addWidget(QLabel("EE Pitch deg:\nStandard EE pitch is 10"))
        self.ee_pitch_combo = QComboBox()
        self.ee_pitch_combo.currentIndexChanged.connect(self.update_plot)
        param_layout.addWidget(self.ee_pitch_combo)
        
        # Spiral
        param_layout.addWidget(QLabel("Spiral:\nThis is the target trajectory direction. 0 is EE pointing straight down"))
        self.spiral_combo = QComboBox()
        self.spiral_combo.currentIndexChanged.connect(self.update_plot)
        param_layout.addWidget(self.spiral_combo)
        
        # Right Hand checkbox
        self.righthand_checkbox = QCheckBox("Right Hand")
        self.righthand_checkbox.setChecked(True)
        self.righthand_checkbox.stateChanged.connect(self.update_plot)
        param_layout.addWidget(self.righthand_checkbox)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # Statistics
        self.stats_group = QGroupBox("Statistics")
        stats_layout = QVBoxLayout()
        self.stats_label = QLabel("No data loaded")
        stats_layout.addWidget(self.stats_label)
        self.stats_group.setLayout(stats_layout)
        layout.addWidget(self.stats_group)
        
        # Refresh button
        self.refresh_button = QPushButton("Force Refresh Plot")
        self.refresh_button.clicked.connect(self.update_plot)
        layout.addWidget(self.refresh_button)
        
        layout.addStretch()
        return panel
    
    def create_plot_panel(self):
        """Create right panel with matplotlib 3D plot"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Create matplotlib figure
        self.figure = Figure(figsize=(10, 8))
        self.canvas = FigureCanvas(self.figure)
        
        # Add navigation toolbar
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        
        return panel
    
    def load_csv(self):
        """Load CSV file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open CSV File", "", "CSV Files (*.csv);;All Files (*)"
        )
        
        if file_path:
            try:
                self.df = pd.read_csv(file_path)
                
                # Calculate spiral if not present
                if 'spiral' not in self.df.columns:
                    self.df['spiral'] = np.arccos(
                        np.clip(self.df['targetDir_z'], -1, 1)
                    ) / np.radians(135.0)
                    self.df['spiral'] = self.df['spiral'].round(2)
                
                self.file_label.setText(f"Loaded: {file_path.split('/')[-1]}")
                self.statusBar.showMessage(f"Loaded {len(self.df)} poses")
                
                # Populate combo boxes
                self.populate_controls()
                
                # Update plot
                self.update_plot()
                
            except Exception as e:
                self.statusBar.showMessage(f"Error loading file: {str(e)}")
    
    def populate_controls(self):
        """Populate combo boxes with unique values"""
        if self.df is None:
            return
        
        # EE Roll
        ee_roll_values = sorted(self.df['EE_roll_deg'].unique())
        self.ee_roll_combo.clear()
        self.ee_roll_combo.addItems([str(v) for v in ee_roll_values])
        self.ee_roll_combo.setCurrentText(str(ee_roll_values[0]))
        
        # EE Pitch
        ee_pitch_values = sorted(self.df['EE_pitch_deg'].unique())
        self.ee_pitch_combo.clear()
        self.ee_pitch_combo.addItems([str(v) for v in ee_pitch_values])
        self.ee_pitch_combo.setCurrentText(str(ee_pitch_values[0]))
        
        # Spiral
        spiral_values = sorted(self.df['spiral'].unique())
        self.spiral_combo.clear()
        self.spiral_combo.addItems([f"{v:.2f}" for v in spiral_values])
        self.spiral_combo.setCurrentText(f"{spiral_values[0]:.2f}")

    def draw_robot_geometry(self, ax):
        """Draw robot base (box) and vertical axis (cylinder)"""
        
        # === ROBOT BASE (Box) ===
        # Define base dimensions (adjust to your robot)
        base_length = 800  # mm (X direction)
        base_width = 150   # mm (Y direction)
        base_height = 500   # mm (Z direction)
        base_center = [-base_length/2 + 100, 0, -base_height/2]  # Center at origin, bottom at z=0
        
        # Create box vertices
        x_base = base_center[0] + np.array([-1, 1, 1, -1, -1, 1, 1, -1]) * base_length/2
        y_base = base_center[1] + np.array([-1, -1, 1, 1, -1, -1, 1, 1]) * base_width/2
        z_base = base_center[2] + np.array([-1, -1, -1, -1, 1, 1, 1, 1]) * base_height/2
        
        # Define the 6 faces of the box
        vertices = [list(zip(x_base, y_base, z_base))]
        
        # Draw box faces
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        # Define faces
        faces = [
            [vertices[0][0], vertices[0][1], vertices[0][2], vertices[0][3]],  # Bottom
            [vertices[0][4], vertices[0][5], vertices[0][6], vertices[0][7]],  # Top
            [vertices[0][0], vertices[0][1], vertices[0][5], vertices[0][4]],  # Front
            [vertices[0][2], vertices[0][3], vertices[0][7], vertices[0][6]],  # Back
            [vertices[0][0], vertices[0][3], vertices[0][7], vertices[0][4]],  # Left
            [vertices[0][1], vertices[0][2], vertices[0][6], vertices[0][5]],  # Right
        ]
        
        box = Poly3DCollection(faces, alpha=0.3, facecolor='gray', edgecolor='black', linewidth=1)
        ax.add_collection3d(box)
        
        # === VERTICAL AXIS (Cylinder) ===
        # Cylinder parameters
        cylinder_radius = 50   # mm
        cylinder_height = 500  # mm (adjust to your vertical range)
        cylinder_base_z = 0    # Start at z=0
        
        # Create cylinder mesh
        theta = np.linspace(0, 2*np.pi, 30)
        z_cyl = np.linspace(cylinder_base_z, cylinder_base_z + cylinder_height, 50)
        Theta, Z_cyl = np.meshgrid(theta, z_cyl)
        
        X_cyl = cylinder_radius * np.cos(Theta)
        Y_cyl = cylinder_radius * np.sin(Theta)
        
        # Plot cylinder surface
        ax.plot_surface(X_cyl, Y_cyl, Z_cyl, alpha=0.2, color='lightblue', 
                    edgecolor='none', shade=True)
        
        # Add cylinder caps (top and bottom circles)
        # Bottom cap
        theta_cap = np.linspace(0, 2*np.pi, 30)
        r_cap = np.linspace(0, cylinder_radius, 10)
        Theta_cap, R_cap = np.meshgrid(theta_cap, r_cap)
        X_cap_bottom = R_cap * np.cos(Theta_cap)
        Y_cap_bottom = R_cap * np.sin(Theta_cap)
        Z_cap_bottom = np.full_like(X_cap_bottom, cylinder_base_z)
        ax.plot_surface(X_cap_bottom, Y_cap_bottom, Z_cap_bottom, alpha=0.2, color='lightblue')
        
        # Top cap
        Z_cap_top = np.full_like(X_cap_bottom, cylinder_base_z + cylinder_height)
        ax.plot_surface(X_cap_bottom, Y_cap_bottom, Z_cap_top, alpha=0.2, color='lightblue')
        
        # Add central axis line for visibility
        ax.plot([0, 0], [0, 0], [cylinder_base_z, cylinder_base_z + cylinder_height],
            'b--', linewidth=2, label='Vertical Axis')
    
    def update_plot(self):
        """Update 3D plot based on current slider values"""
        if self.df is None:
            return
        
        try:
            # Get current parameter values
            ee_roll = float(self.ee_roll_combo.currentText())
            ee_pitch = float(self.ee_pitch_combo.currentText())
            spiral = float(self.spiral_combo.currentText())
            righthand = 1 if self.righthand_checkbox.isChecked() else 0
            
            # Filter data
            mask = (
                (self.df['EE_roll_deg'] == ee_roll) &
                (self.df['EE_pitch_deg'] == ee_pitch) &
                (np.abs(self.df['spiral'] - spiral) < 0.01) &
                (self.df['rightHand'] == righthand)
            )
            
            self.filtered_df = self.df[mask]
            
            # Update statistics
            self.update_statistics()

            # Save prev view
            if self.ax is not None:
                self.prev_view['elev'] = self.ax.elev
                self.prev_view['azim'] = self.ax.azim
                self.prev_view['roll'] = self.ax.roll
            
            # Clear and redraw plot
            self.figure.clear()
            self.ax = self.figure.add_subplot(111, projection='3d')

            self.draw_robot_geometry(self.ax)
            
            if len(self.filtered_df) == 0:
                self.ax.text(0, 0, 0, "No reachable poses\nfor these parameters",
                       fontsize=14, ha='center', color='red')
                self.ax.set_xlabel('X (mm)')
                self.ax.set_ylabel('Y (mm)')
                self.ax.set_zlabel('Z (mm)')
            else:
                # Plot points
                positions = self.filtered_df[['x', 'y', 'z']].values
                directions = self.filtered_df[['targetDir_x', 'targetDir_y', 'targetDir_z']].values
                
                self.ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
                      c='blue', marker='o', s=10, alpha=0.6, 
                      rasterized=True)
                
                
                # Get the direction (same for all points in this filtered set)
                direction = directions[0]  # All directions are the same for this spiral value
                
                # Draw one large arrow
                arrow_length = 300  # Larger arrow for visibility
                self.ax.quiver(0, 0, 0,
                        direction[0], direction[1], direction[2],
                        length=arrow_length, color='red', alpha=0.8,
                        arrow_length_ratio=0.3, linewidth=3,
                        normalize=False, label='Target Direction')
                
                self.ax.set_xlabel('X (mm)')
                self.ax.set_ylabel('Y (mm)')
                self.ax.set_zlabel('Z (mm)')
                
                # Set axis limits
                self.ax.set_xlim(-1000, 1000)
                self.ax.set_ylim(-1000, 1000)
                self.ax.set_zlim(-1000, 1000)

            # Restore previous view
            self.ax.view_init(elev=self.prev_view['elev'], azim=self.prev_view['azim'], roll=self.prev_view['roll'])
            
            self.canvas.draw()
            
        except Exception as e:
            self.statusBar.showMessage(f"Error updating plot: {str(e)}")
    
    def update_statistics(self):
        """Update statistics panel"""
        if self.filtered_df is not None:
            stats_text = f"Filtered Poses: {len(self.filtered_df)}\n"
            if len(self.filtered_df) > 0:
                stats_text += f"\nX range: [{self.filtered_df['x'].min():.1f}, {self.filtered_df['x'].max():.1f}] mm\n"
                stats_text += f"Y range: [{self.filtered_df['y'].min():.1f}, {self.filtered_df['y'].max():.1f}] mm\n"
                stats_text += f"Z range: [{self.filtered_df['z'].min():.1f}, {self.filtered_df['z'].max():.1f}] mm\n"
            self.stats_label.setText(stats_text)

def main():
    app = QApplication(sys.argv)
    window = WorkspaceVisualizerApp()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()