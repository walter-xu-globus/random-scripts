import matplotlib.pyplot as plt
import re

with open("2/normal2.txt", "r") as file:
    lines = file.readlines()

    pos = {"v": [], "s": [], "e": [], "r": [], "p": []}
    vel = {"v": [], "s": [], "e": [], "r": [], "p": []} 
    straight_line_ind = 0

    for i, line in enumerate(lines):
        if line.startswith("POS"):
            pattern = re.escape("v: ") + "(.*?)" + re.escape(",")
            pos["v"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("s: ") + "(.*?)" + re.escape(",")
            pos["s"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("e: ") + "(.*?)" + re.escape(",")
            pos["e"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("r: ") + "(.*?)" + re.escape(",")
            pos["r"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("p: ") + "(.*?)" + re.escape("}")
            pos["p"].append(float(re.findall(pattern, line)[0]))
        elif line.startswith("VEL"):
            pattern = re.escape("v: ") + "(.*?)" + re.escape(",")
            vel["v"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("s: ") + "(.*?)" + re.escape(",")
            vel["s"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("e: ") + "(.*?)" + re.escape(",")
            vel["e"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("r: ") + "(.*?)" + re.escape(",")
            vel["r"].append(float(re.findall(pattern, line)[0]))
            pattern = re.escape("p: ") + "(.*?)" + re.escape("}")
            vel["p"].append(float(re.findall(pattern, line)[0]))
        elif line.startswith("MTN"):
            straight_line_ind = i//2

fig1, ax1 = plt.subplots()
ax1t = ax1.twinx()
ax1.plot(pos["v"], color="r")
ax1.set_ylabel("pos", color="r")
ax1t.plot(vel["v"], color="b")
ax1t.set_ylabel("vel", color="b")
ax1.set_title("vertical")
ax1.axvline(straight_line_ind)

fig2, ax2 = plt.subplots()
ax2t = ax2.twinx()
ax2.plot(pos["s"], color="r")
ax2.set_ylabel("pos", color="r")
ax2t.plot(vel["s"], color="b")
ax2t.set_ylabel("vel", color="b")
ax2.set_title("shoulder")
ax2.axvline(straight_line_ind)

fig3, ax3 = plt.subplots()
ax3t = ax3.twinx()
ax3.plot(pos["e"], color="r")
ax3.set_ylabel("pos", color="r")
ax3t.plot(vel["e"], color="b")
ax3t.set_ylabel("vel", color="b")
ax3.set_title("elbow")
ax3.axvline(straight_line_ind)

fig4, ax4 = plt.subplots()
ax4t = ax4.twinx()
ax4.plot(pos["r"], color="r")
ax4.set_ylabel("pos", color="r")
ax4t.plot(vel["r"], color="b")
ax4t.set_ylabel("vel", color="b")
ax4.set_title("roll")
ax4.axvline(straight_line_ind)

fig5, ax5 = plt.subplots()
ax5t = ax5.twinx()
ax5.plot(pos["p"], color="r")
ax5.set_ylabel("pos", color="r")
ax5t.plot(vel["p"], color="b")
ax5t.set_ylabel("vel", color="b")
ax5.set_title("pitch")
ax5.axvline(straight_line_ind)

plt.show()

#-------------------------------------------------------------3d trajectory plot

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

time = np.linspace(0, 10, 100)
x = np.sin(time)
y = np.cos(time)
z = time

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(x, y, z, c=time, cmap='viridis', label='Trajectory')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
plt.title('3D Position Trajectory')
plt.colorbar(sc, label='Time (s)')
plt.show()


