import matplotlib.pyplot as plt
import numpy as np

# Dane do wykresów
voxel_sizes = [0.089, 0.078, 0.067, 0.056, 0.045, 0.023, 0.012, 0.001]
times_zed = [3.63, 3.71, 3.59, 3.68, 3.96, 5.57, 30.88]
times_realsense = [0.10, 0.11, 0.09, 0.10, 0.10, 0.09, 0.12, 0.34]

# 1. Wykres liniowy dla kamery ZED
plt.figure(figsize=(10, 6))
plt.plot(voxel_sizes[:len(times_zed)], times_zed, marker='o', color='blue', label="Kamera ZED", linestyle='-')
plt.xlabel("Rozmiar voxela", fontsize=12)
plt.ylabel("Czas operacji (s)", fontsize=12)
plt.title("Zależność czasu operacji od rozmiaru voxela - Kamera ZED", fontsize=14)
plt.gca().invert_xaxis()  # Odwrócenie osi X, bo mniejszy voxel oznacza więcej punktów
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend(fontsize=12)
plt.tight_layout()
plt.savefig("voxel_vs_time_zed.png")  # Zapisz wykres jako plik PNG
plt.show()

# 2. Wykres liniowy dla kamery RealSense
plt.figure(figsize=(10, 6))
plt.plot(voxel_sizes, times_realsense, marker='o', color='green', label="Kamera RealSense", linestyle='--')
plt.xlabel("Rozmiar voxela", fontsize=12)
plt.ylabel("Czas operacji (s)", fontsize=12)
plt.title("Zależność czasu operacji od rozmiaru voxela - Kamera RealSense", fontsize=14)
plt.gca().invert_xaxis()  # Odwrócenie osi X
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend(fontsize=12)
plt.tight_layout()
plt.savefig("voxel_vs_time_realsense.png")  # Zapisz wykres jako plik PNG
plt.show()
