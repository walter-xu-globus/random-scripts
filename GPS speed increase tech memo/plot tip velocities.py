import matplotlib.pyplot as plt
import numpy as np
import re

def read(controller_file_name, sensorhub_file_name):
    with open(controller_file_name, "r") as file:
        lines = file.readlines()
    with open(sensorhub_file_name, "r") as file:
        lines = lines + file.readlines()

    controller_time = []
    controller_pos = {"x": [], "y": [], "z": []}
    sensorhub_time = []
    sensorhub_vel = {"x": [], "y": [], "z": []}
    trajectory_mode_start_time = 0
    trajectory_mode_end_time = 0

    for i, line in enumerate(lines):
        if line.startswith("Time EE pos"):
            pattern = re.escape("Time EE pos(ms): ") + "(.*?)" + "\n"
            controller_time.append(int(re.findall(pattern, line)[0]))
        elif line.startswith("EE pos"):
            pattern = re.escape("EE pos = [") + "(.*?)" + re.escape("]")
            s = re.findall(pattern, line)[0]
            s = s.split(", ")
            controller_pos["x"].append(float(s[0]))
            controller_pos["y"].append(float(s[1]))
            controller_pos["z"].append(float(s[2]))
        elif line.startswith("MTN: Move Mode -TRAJECTORY_MODE"):
            trajectory_mode_start_time = controller_time[-1] - controller_time[0]
        elif line.startswith("Time Sensorhub (ms):"):
            s = line.split(" ")
            if s[6] != "2": continue # ID 2 is end effector
            sensorhub_time.append(int(s[3]))
            pattern = re.escape("linear velocity: [") + "(.*?)" + re.escape("]")
            s = re.findall(pattern, line)[0]
            s = s.split(", ")
            sensorhub_vel["x"].append(float(s[0]))
            sensorhub_vel["y"].append(float(s[1]))
            sensorhub_vel["z"].append(float(s[2]))

    # convert to np
    controller_pos = np.vstack((np.array(controller_pos["x"]), np.array(controller_pos["y"]), np.array(controller_pos["z"])))
    controller_time = np.array(controller_time)
    sensorhub_vel = np.vstack((np.array(sensorhub_vel["x"]), np.array(sensorhub_vel["y"]), np.array(sensorhub_vel["z"])))
    sensorhub_time = np.array(sensorhub_time)

    # sync time
    while controller_time[0] > sensorhub_time[0]: 
        sensorhub_time = sensorhub_time[1:]
        sensorhub_vel = sensorhub_vel[:, 1:]
    sensorhub_time -= controller_time[0]
    controller_time -= controller_time[0]
    
    trajectory_mode_end_time = controller_time[-1]

    # calculate controller and sensorhub vel
    delta_s = np.diff(controller_pos, axis=1, append=controller_pos[:,[-1]])
    mask = delta_s != 0
    delta_s = delta_s[:, np.max(mask,keepdims=False, axis=0)]
    delta_t = np.diff(controller_time[np.max(mask,keepdims=False, axis=0)], append=2*controller_time[-1] - controller_time[-2]) / 1000
    controller_vel = delta_s/delta_t
    controller_time = controller_time[np.max(mask,keepdims=False, axis=0)]

    controller_vel = np.linalg.norm(controller_vel, ord=2, axis=0)
    sensorhub_vel = np.linalg.norm(sensorhub_vel, ord=2, axis=0)

    # controller vel needs filtering
    window_size = 2
    K = np.ones(window_size) * 1/window_size
    controller_vel = np.convolve(controller_vel, K, mode="same")
    
    assert(controller_vel.shape[0] == controller_time.shape[0])
    assert(sensorhub_vel.shape[0] == sensorhub_time.shape[0])

    data = Data(controller_vel, controller_time, sensorhub_vel, sensorhub_time, trajectory_mode_start_time, trajectory_mode_end_time)
    return data

def plot(data, name):
    ''' shapes:
        controller_vel [N,]
        controller_time [N,]
        sensorhub_vel [N,]
        sensorhub_time [N,]

    '''
    fig, ax = plt.subplots(1, 1, figsize=(10,6))

    # ax.plot(data.controller_time, data.controller_vel, color="r", label="Tip velocity given by GMAS")
    ax.plot(data.sensorhub_time, data.sensorhub_vel, color="g", label="Tip velocity given by Sensorhub")

    ax.axvline(data.trajectory_mode_start_time, linestyle=":", color="k", label="Gravity mode end, changing to trajectory mode")
    ax.axvline(data.trajectory_mode_end_time, linestyle=":", color="k", label="Trajectory mode end, target reached")
    ax.set_box_aspect(0.5)
    ax.set_xlabel("time (ms)")
    ax.set_ylabel("Tip velocity (mm/s)")
    ax.set_title("Tip Velocity " + name)

    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right')
    

    plt.show()

class Data:
    def __init__(self, controller_vel, controller_time, sensorhub_vel, sensorhub_time, trajectory_mode_start_time, trajectory_mode_end_time):
        self.controller_vel = controller_vel
        self.controller_time = controller_time
        self.sensorhub_vel = sensorhub_vel
        self.sensorhub_time = sensorhub_time
        self.trajectory_mode_start_time = trajectory_mode_start_time
        self.trajectory_mode_end_time = trajectory_mode_end_time

if __name__ == "__main__":
    name = "L5R to L5L"
    data = read(name + "/tip velocity plot/mine controller.txt", name + "/tip velocity plot/mine sensorhub.txt")
    plot(data, name)