import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import json
import time

def capture_and_process_intrinsics(output_ply_filename, output_json_filename, duration=10):
    # Inicjalizacja pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Włączanie strumieni RGB i głębi
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Strumień RGB
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # Strumień głębi

    # Uruchomienie pipeline
    profile = pipeline.start(config)

    # Pobieranie strumienia kamer
    color_stream = profile.get_stream(rs.stream.color)
    depth_stream = profile.get_stream(rs.stream.depth)

    # Pobieranie parametrów intrystycznych dla obu kamer
    color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
    depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

    # Zapisujemy intrinseki do pliku JSON
    intrinsics_data = {
        "color": {
            "width": color_intrinsics.width,
            "height": color_intrinsics.height,
            "fx": color_intrinsics.fx,
            "fy": color_intrinsics.fy,
            "cx": color_intrinsics.ppx,
            "cy": color_intrinsics.ppy,
            "coeffs": color_intrinsics.coeffs
        },
        "depth": {
            "width": depth_intrinsics.width,
            "height": depth_intrinsics.height,
            "fx": depth_intrinsics.fx,
            "fy": depth_intrinsics.fy,
            "cx": depth_intrinsics.ppx,
            "cy": depth_intrinsics.ppy,
            "coeffs": depth_intrinsics.coeffs
        }
    }

    # Zapisujemy intrinseki do pliku JSON
    with open(output_json_filename, 'w') as json_file:
        json.dump(intrinsics_data, json_file, indent=4)

    print(f"Intrinseki zapisane w pliku: {output_json_filename}")

    # Inicjalizacja chmury punktów
    pc = rs.pointcloud()
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Rozpoczynamy pomiar czasu
    start_time = time.time()

    try:
        # Pętla do zbierania i przetwarzania klatek przez określony czas
        while True:
            # Sprawdzamy, czy minął określony czas (np. 10 sekund)
            elapsed_time = time.time() - start_time
            if elapsed_time > duration:
                print(f"Czas nagrywania minął ({duration} sekund). Zakończono zbieranie danych.")
                break

            # Czekamy na nowe klatki
            frames = pipeline.wait_for_frames()

            # Wyrównanie strumieni głębokości i kolorowego
            aligned_frames = align.process(frames)

            # Pobieramy klatki
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Mapowanie głębi na obraz RGB
            pc.map_to(color_frame)
            pointcloud = pc.calculate(depth_frame)

            # Przekształcenie chmury punktów na tablicę NumPy
            vertices = np.asanyarray(pointcloud.get_vertices())  # Współrzędne punktów 3D (X, Y, Z)
            colors = np.asanyarray(color_frame.get_data())  # Kolory z kamery RGB

            # Tworzymy chmurę punktów Open3D
            points = np.array([ [point[0], point[1], point[2]] for point in vertices])  # Współrzędne punktów
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # Normalizacja kolorów (z zakresu [0, 255] do [0, 1])
            rgb = colors.reshape(-1, 3) / 255.0
            pcd.colors = o3d.utility.Vector3dVector(rgb)

            # Zapisujemy chmurę punktów do pliku PLY
            o3d.io.write_point_cloud(output_ply_filename, pcd)
            print(f"Chmura punktów zapisana do {output_ply_filename}")

    finally:
        # Zatrzymanie pipeline po zakończeniu
        pipeline.stop()

if __name__ == "__main__":
    # Ścieżki plików
    output_ply_filename = "point_cloud.ply"
    output_json_filename = "intrinsics.json"
    duration = 10  # Czas w sekundach, przez jaki program ma zbierać dane

    # Wywołanie funkcji
    capture_and_process_intrinsics(output_ply_filename, output_json_filename, duration)
