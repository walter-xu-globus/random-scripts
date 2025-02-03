import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import re

def read(file_name):
    with open(file_name, "r") as file:
        lines = file.readlines()

    axis_pos = {"v": [], "s": [], "e": [], "r": [], "p": []}
    axis_vel = {"v": [], "s": [], "e": [], "r": [], "p": []} 
    axis_time = []
    ee_time = []
    ee_pos = {"x": [], "y": [], "z": []}
    trajectory_mode_start_time = 0

    for i, line in enumerate(lines):
        if line.startswith("Time EE pos"):
            pattern = re.escape("Time EE pos(ms): ") + "(.*?)" + "\n"
            ee_time.append(int(re.findall(pattern, line)[0]))
        elif line.startswith("EE pos"):
            pattern = re.escape("EE pos = [") + "(.*?)" + re.escape("]")
            s = re.findall(pattern, line)[0]
            s = s.split(", ")
            ee_pos["x"].append(float(s[0]))
            ee_pos["y"].append(float(s[1]))
            ee_pos["z"].append(float(s[2]))
        elif line.startswith("Time axis"):
            pattern = re.escape("Time axis (ms): ") + "(.*?)" + "\n"
            axis_time.append(int(re.findall(pattern, line)[0]))
        elif line.startswith("POS"):
            pattern = re.escape("v: ") + "(.*?)" + re.escape(",")
            axis_pos["v"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("s: ") + "(.*?)" + re.escape(",")
            axis_pos["s"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("e: ") + "(.*?)" + re.escape(",")
            axis_pos["e"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("r: ") + "(.*?)" + re.escape(",")
            axis_pos["r"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("p: ") + "(.*?)" + re.escape("}")
            axis_pos["p"].append(float(re.findall(pattern, line)[0]))
        elif line.startswith("VEL"):
            pattern = re.escape("v: ") + "(.*?)" + re.escape(",")
            axis_vel["v"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("s: ") + "(.*?)" + re.escape(",")
            axis_vel["s"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("e: ") + "(.*?)" + re.escape(",")
            axis_vel["e"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("r: ") + "(.*?)" + re.escape(",")
            axis_vel["r"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("p: ") + "(.*?)" + re.escape("}")
            axis_vel["p"].append(float(re.findall(pattern, line)[0]))
        elif line.startswith("MTN: Move Mode -TRAJECTORY_MODE"):
            trajectory_mode_start_time = ee_time[-1] - ee_time[0]

    trajectory_mode_end_time = ee_time[-1] - ee_time[0]

    axis_pos = np.vstack((np.array(axis_pos["v"]), np.array(axis_pos["s"]), np.array(axis_pos["e"]), np.array(axis_pos["r"]), np.array(axis_pos["p"])))
    axis_vel = np.vstack((np.array(axis_vel["v"]), np.array(axis_vel["s"]), np.array(axis_vel["e"]), np.array(axis_vel["r"]), np.array(axis_vel["p"])))
    axis_time = np.array(axis_time) - axis_time[0]
    ee_pos = np.vstack((np.array(ee_pos["x"]), np.array(ee_pos["y"]), np.array(ee_pos["z"])))
    ee_time = np.array(ee_time) - ee_time[0]
    

    assert(axis_time.shape[0] == axis_pos.shape[1] == axis_vel.shape[1])
    assert(ee_time.shape[0] == ee_pos.shape[1])

    data = Data(axis_pos, axis_vel, axis_time, ee_pos, ee_time, trajectory_mode_start_time, trajectory_mode_end_time)
    return data

def plot(mine, original, name):
    ''' shapes:
        axis_pos [5,N]
        axis_vel [5,N]
        axis_time [N,]
        ee_pos [3,N]
        ee_time [N,]
    '''
    fig, ax = plt.subplots(5, 2, sharex="row", figsize=(15, 50))
    fig.suptitle(name, fontsize=20)
    for axis_num in range(5):
        ax[axis_num, 0].plot(mine.axis_time, mine.axis_pos[axis_num, :], color="r", label="Proposed")
        ax[axis_num, 0].plot(original.axis_time, original.axis_pos[axis_num, :], color="g", label="Original")
        ax[axis_num, 0].axvline(mine.trajectory_mode_start_time, linestyle=":", color="r", label="Gravity mode end, changing to trajectory mode")
        ax[axis_num, 0].axvline(original.trajectory_mode_start_time, linestyle=":", color="g", label="Gravity mode end, changing to trajectory mode")
        ax[axis_num, 0].axvline(mine.trajectory_mode_end_time, linestyle=":", color="r", label="Trajectory mode end, target reached")
        ax[axis_num, 0].axvline(original.trajectory_mode_end_time, linestyle=":", color="g", label="Trajectory mode end, target reached")
        ax[axis_num, 0].set_box_aspect(0.5)
        ax[axis_num, 0].set_xlabel("time (ms)")

    for axis_num in range(5):
        ax[axis_num, 1].plot(mine.axis_time, mine.axis_vel[axis_num, :], color="r", label="Proposed")
        ax[axis_num, 1].plot(original.axis_time, original.axis_vel[axis_num, :], color="g", label="Original")
        ax[axis_num, 1].axvline(mine.trajectory_mode_start_time, linestyle=":", color="r", label="Gravity mode end, changing to trajectory mode")
        ax[axis_num, 1].axvline(original.trajectory_mode_start_time, linestyle=":", color="g", label="Gravity mode end, changing to trajectory mode")
        ax[axis_num, 1].axvline(mine.trajectory_mode_end_time, linestyle=":", color="r", label="Trajectory mode end, target reached")
        ax[axis_num, 1].axvline(original.trajectory_mode_end_time, linestyle=":", color="g", label="Trajectory mode end, target reached")
        ax[axis_num, 1].set_box_aspect(0.5)
        ax[axis_num, 1].set_xlabel("time (ms)")


    handles, labels = ax[-1,-1].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right')

    fig.text(0.04, 0.8, 'Vertical(mm)', ha='center', fontsize=12, rotation='vertical')
    fig.text( 0.04, 0.62, 'Shoulder(deg)', ha='center', fontsize=12, rotation='vertical')
    fig.text(0.04, 0.48, 'Elbow(deg)', ha='center', fontsize=12, rotation='vertical')
    fig.text(0.04, 0.33, 'Roll(deg)', ha='center', fontsize=12, rotation='vertical')
    fig.text(0.04, 0.15, 'Pitch(deg)', ha='center', fontsize=12, rotation='vertical')
    fig.text(0.7, 0.9,'Velocity', va='center',  fontsize=15)
    fig.text(0.2, 0.9, 'Position', va='center', fontsize=15)

    fig2 = plt.figure()
    ax = fig2.add_subplot(111, projection='3d')
    sc = ax.scatter(mine.ee_pos[0], mine.ee_pos[1], mine.ee_pos[2], c=mine.ee_time, cmap='viridis', label='Trajectory')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.title('EE tip position (mine)')
    plt.colorbar(sc, label='Time (ms)')
    

    fig3 = plt.figure()
    ax = fig3.add_subplot(111, projection='3d')
    sc = ax.scatter(original.ee_pos[0], original.ee_pos[1], original.ee_pos[2], c=original.ee_time, cmap='viridis', label='Trajectory')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.title('EE tip position (original)')
    plt.colorbar(sc, label='Time (ms)')

    plt.show()

class Data:
    def __init__(self, axis_pos, axis_vel, axis_time, ee_pos, ee_time, trajectory_mode_start_time, trajectory_mode_end_time):
        self.axis_pos = axis_pos
        self.axis_vel = axis_vel
        self.axis_time = axis_time
        self.ee_pos = ee_pos
        self.ee_time = ee_time
        self.trajectory_mode_start_time = trajectory_mode_start_time
        self.trajectory_mode_end_time = trajectory_mode_end_time

if __name__ == "__main__":
    name = "L5R to L1L"

    mine = read(name + "/mine.txt")
    original = read(name + "/original.txt")

    # mine = read(name + "/mine 2nd try.txt")
    # original = read(name + "/original 2nd try.txt")

    # mine = read(name + "/mine 3rd try.txt")
    # original = read(name + "/original 3rd try.txt")
    plot(mine, original, name)