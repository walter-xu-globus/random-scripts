import matplotlib.pyplot as plt
import csv

with open("wtf.txt", "r") as file:
    reader = csv.DictReader(file)
    pos_error = []
    current_command = []
    for row in reader:
        pos_error.append(float(row["pos_error"]))
        current_command.append(float(row["current_command"]))
	

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    ax1.plot(pos_error, color="r")
    ax1.set_ylabel("pos error (counts)", color="r")
    ax2.plot(current_command, color="b")
    ax2.set_ylabel("current command (A)", color="b")
    plt.show()

