import matplotlib.pyplot as plt
import numpy as np

# Data categories
categories = ["L1L to L1R", "L1L to L4L", "L1L to L5R", "L5R to L1L", "L5R to L4R", "L5R to L5L"]

# Gravity mode times (ms)
gravity_proposed = [4962.3, 2821.7, 5118.5, 5239.0, 3444.7, 5134.0]
gravity_original = [6799.0, 4682.0, 7450.7, 7376.0, 4778.0, 7187.3]

# Trajectory mode times (ms)
trajectory_proposed = [7986.3, 8861.0, 7979.5, 8129.7, 8272.0, 8233.7]
trajectory_original = [10192.7, 10447.0, 10792.0, 10269.0, 10894.5, 10284.0]

# Overall completion times (ms)
overall_proposed = [12948.7, 11682.7, 13098.0, 13368.7, 11716.7, 13367.7]
overall_original = [16991.7, 15129.0, 18242.7, 17645.0, 15672.5, 17471.3]

# Bar chart parameters
x = np.arange(len(categories))
width = 0.2

# Adjust legend placement to avoid overlapping bars
fig, ax = plt.subplots(figsize=(12, 6))

# Adjusted x locations for better spacing
x = np.arange(len(categories)) * 1.5

# Plot bars with increased spacing
ax.bar(x - width, gravity_proposed, width, label='Gravity Proposed', color='b', alpha=0.7)
ax.bar(x, gravity_original, width, label='Gravity Original', color='b', hatch='//', alpha=0.7)

ax.bar(x + width, trajectory_proposed, width, label='Trajectory Proposed', color='g', alpha=0.7)
ax.bar(x + 2 * width, trajectory_original, width, label='Trajectory Original', color='g', hatch='//', alpha=0.7)

ax.bar(x + 3 * width, overall_proposed, width, label='Overall Proposed', color='r', alpha=0.7)
ax.bar(x + 4 * width, overall_original, width, label='Overall Original', color='r', hatch='//', alpha=0.7)

# Formatting the chart
ax.set_xlabel("Transition Movements")
ax.set_ylabel("Completion Time (ms)")
ax.set_title("Comparison of Completion Times (Proposed vs. Original)")
ax.set_xticks(x + width)
ax.set_xticklabels(categories, rotation=20, ha="right")

# Move legend outside the plot
ax.legend(loc='upper left', bbox_to_anchor=(1, 1))

plt.tight_layout()
plt.show()
