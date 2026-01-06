import numpy as np
import matplotlib.pyplot as plt
import os
from mpl_toolkits.mplot3d import Axes3D

# Read the CSV files
reachable_path = r"C:\Development\gh_inr_dev\build\INR_DEV\Clang-D\Testing\IK_sweep_reachable.csv"
unreachable_path = r"C:\Development\gh_inr_dev\build\INR_DEV\Clang-D\Testing\IK_sweep_unreachable.csv"
try:
    reachable = np.loadtxt(reachable_path, delimiter=',', skiprows=1)
    unreachable = np.loadtxt(unreachable_path, delimiter=',', skiprows=1)
    
    print(f"Reachable points: {len(reachable)}")
    print(f"Unreachable points: {len(unreachable)}")
    
except FileNotFoundError as e:
    print(f"Error: Could not find CSV file - {e}")
    exit(1)

# Create 3D plot
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot reachable points (green)
if len(reachable) > 0:
    ax.scatter(reachable[:, 0], reachable[:, 1], reachable[:, 2], 
               c='green', marker='.', s=1, alpha=1.0, label='Reachable')

# # Plot unreachable points (red)
# if len(unreachable) > 0:
#     ax.scatter(unreachable[:, 0], unreachable[:, 1], unreachable[:, 2], 
#                c='red', marker='.', s=1, alpha=1.0, label='Unreachable')

# Labels and formatting
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('reachable workspace points (green)')
ax.legend()

# Set equal aspect ratio
max_range = 900
ax.set_xlim([-max_range, max_range])
ax.set_ylim([-max_range, max_range])
ax.set_zlim([-200, 600])

plt.tight_layout()
plt.savefig('workspace_3d.png', dpi=300)
plt.show()

# Create 2D slices
fig, axes = plt.subplots(2, 2, figsize=(14, 12))

# XY plane at Z=200mm
z_slice = 200
if len(reachable) > 0:
    mask_r = np.abs(reachable[:, 2] - z_slice) < 25  # ±25mm tolerance
    axes[0, 0].scatter(reachable[mask_r, 0], reachable[mask_r, 1], 
                       c='green', s=5, alpha=0.5, label='Reachable')
if len(unreachable) > 0:
    mask_u = np.abs(unreachable[:, 2] - z_slice) < 25
    axes[0, 0].scatter(unreachable[mask_u, 0], unreachable[mask_u, 1], 
                       c='red', s=5, alpha=0.5, label='Unreachable')
axes[0, 0].set_xlabel('X (mm)')
axes[0, 0].set_ylabel('Y (mm)')
axes[0, 0].set_title(f'XY Plane at Z={z_slice}mm')
axes[0, 0].legend()
axes[0, 0].grid(True)
axes[0, 0].axis('equal')

# XZ plane at Y=0mm
y_slice = 0
if len(reachable) > 0:
    mask_r = np.abs(reachable[:, 1] - y_slice) < 25
    axes[0, 1].scatter(reachable[mask_r, 0], reachable[mask_r, 2], 
                       c='green', s=5, alpha=0.5, label='Reachable')
if len(unreachable) > 0:
    mask_u = np.abs(unreachable[:, 1] - y_slice) < 25
    axes[0, 1].scatter(unreachable[mask_u, 0], unreachable[mask_u, 2], 
                       c='red', s=5, alpha=0.5, label='Unreachable')
axes[0, 1].set_xlabel('X (mm)')
axes[0, 1].set_ylabel('Z (mm)')
axes[0, 1].set_title(f'XZ Plane at Y={y_slice}mm')
axes[0, 1].legend()
axes[0, 1].grid(True)
axes[0, 1].axis('equal')

# YZ plane at X=0mm
x_slice = 0
if len(reachable) > 0:
    mask_r = np.abs(reachable[:, 0] - x_slice) < 25
    axes[1, 0].scatter(reachable[mask_r, 1], reachable[mask_r, 2], 
                       c='green', s=5, alpha=0.5, label='Reachable')
if len(unreachable) > 0:
    mask_u = np.abs(unreachable[:, 0] - x_slice) < 25
    axes[1, 0].scatter(unreachable[mask_u, 1], unreachable[mask_u, 2], 
                       c='red', s=5, alpha=0.5, label='Unreachable')
axes[1, 0].set_xlabel('Y (mm)')
axes[1, 0].set_ylabel('Z (mm)')
axes[1, 0].set_title(f'YZ Plane at X={x_slice}mm')
axes[1, 0].legend()
axes[1, 0].grid(True)
axes[1, 0].axis('equal')

# Joint distribution (for reachable points only)
if len(reachable) > 0 and reachable.shape[1] >= 8:
    axes[1, 1].hist(reachable[:, 7], bins=50, alpha=0.7, label='Pitch', color='blue')
    axes[1, 1].set_xlabel('Pitch Angle (degrees)')
    axes[1, 1].set_ylabel('Frequency')
    axes[1, 1].set_title('Pitch Angle Distribution')
    axes[1, 1].legend()
    axes[1, 1].grid(True)

plt.tight_layout()
plt.savefig('../Tests/workspace_slices.png', dpi=300)
plt.show()

print("\nPlots saved as 'workspace_3d.png' and 'workspace_slices.png'")