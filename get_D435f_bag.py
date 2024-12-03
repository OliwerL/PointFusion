import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import time
import os

# Ścieżka do pliku, w którym zapiszemy chmurę punktów
output_ply_file = "C:/Users/ramos/Documents/point_cloud.ply"  # Zmień na odpowiednią ścieżkę

# Sprawdzenie, czy katalog istnieje
output_dir = os.path.dirname(output_ply_file)
if not os.path.exists(output_dir):
    print(f"Katalog {output_dir} nie istnieje. Tworzę go...")
    os.makedirs(output_dir)

# Inicjalizacja pipeline
pipeline = rs.pipeline()

# Inicjalizacja konfiguracji
config = rs.config()

# Włączanie strumieni RGB i głębi
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Strumień RGB
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # Strumień głębi

# Uruchomienie pipeline
pipeline.start(config)

# Czekanie na dane przez kilka sekund
duration = 3  # Czas trwania nagrywania w sekundach
start_time = time.time()

# Inicjalizacja chmury punktów
pc = rs.pointcloud()
frames = None

try:
    while time.time() - start_time < duration:
        # Oczekiwanie na nowe klatki
        frames = pipeline.wait_for_frames()

        # Pobieranie strumienia głębi i RGB
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Generowanie chmury punktów
        pc.map_to(color_frame)  # Mapowanie głębi na obraz RGB
        pointcloud = pc.calculate(depth_frame)  # Obliczenie chmury punktów z głębi

        # Konwersja chmury punktów na tablicę NumPy
        vertices = np.asanyarray(pointcloud.get_vertices())  # Współrzędne punktów

        # 'vertices' jest teraz tablicą z punktami 3D, gdzie kolumny to X, Y, Z
        points = np.array([ [point[0], point[1], point[2]] for point in vertices])  # Konwersja na tablicę 3D

        # Tworzenie obiektu PointCloud z Open3D
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Zapis chmury punktów do pliku PLY
        o3d.io.write_point_cloud(output_ply_file, pcd)

finally:
    # Zatrzymanie pipeline po zakończeniu nagrywania
    pipeline.stop()

print(f"Chmura punktów zapisana w pliku: {output_ply_file}")
