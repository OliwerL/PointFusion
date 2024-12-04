import matplotlib.pyplot as plt
import numpy as np

# Data for the plots
voxel_sizes = [0.089, 0.078, 0.067, 0.056, 0.045, 0.023, 0.012, 0.001]
times_zed = [3.63, 3.71, 3.59, 3.68, 3.96, 5.57, 30.88]
times_realsense = [0.10, 0.11, 0.09, 0.10, 0.10, 0.09, 0.12, 0.34]

# 1. Line plot for ZED camera
plt.figure(figsize=(10, 6))
plt.plot(voxel_sizes[:len(times_zed)], times_zed, marker='o', color='blue', label="ZED Camera", linestyle='-')
plt.xlabel("Voxel Size", fontsize=12)
plt.ylabel("Operation Time (s)", fontsize=12)
plt.title("Operation Time vs Voxel Size - ZED Camera", fontsize=14)
plt.gca().invert_xaxis()  # Invert X-axis because smaller voxels mean more points
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend(fontsize=12)
plt.tight_layout()
plt.savefig("voxel_vs_time_zed.png")  # Save the plot as a PNG file
plt.show()

# 2. Line plot for RealSense camera
plt.figure(figsize=(10, 6))
plt.plot(voxel_sizes, times_realsense, marker='o', color='green', label="RealSense Camera", linestyle='--')
plt.xlabel("Voxel Size", fontsize=12)
plt.ylabel("Operation Time (s)", fontsize=12)
plt.title("Operation Time vs Voxel Size - RealSense Camera", fontsize=14)
plt.gca().invert_xaxis()  # Invert X-axis
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend(fontsize=12)
plt.tight_layout()
plt.savefig("voxel_vs_time_realsense.png")  # Save the plot as a PNG file
plt.show()
