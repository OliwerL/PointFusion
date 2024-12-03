import pyzed.sl as sl
import open3d as o3d
import numpy as np
import time

def save_and_display_point_cloud(zed, filename):
    """ Funkcja zapisująca chmurę punktów i wyświetlająca ją w oknie 3D """
    print("Saving and displaying Point Cloud...")

    # Stworzenie obiektu do przechowywania chmury punktów
    point_cloud = sl.Mat()

    # Pobranie chmury punktów (w formacie XYZRGBA)
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        # Sprawdzenie, czy chmura punktów zawiera dane
        if point_cloud.is_init():
            print(f"Chmura punktów zawiera {point_cloud.get_width() * point_cloud.get_height()} punktów.")

            # Zapisanie chmury punktów do pliku (np. w formacie .ply)
            success = point_cloud.write(filename)
            if success == sl.ERROR_CODE.SUCCESS:
                print(f"Point cloud saved to {filename}")
            else:
                print("Failed to save point cloud.")

            # Konwersja chmury punktów ZED (sl.Mat) na numpy array
            point_cloud_data = point_cloud.get_data()

            # Przekształcenie danych RGB i XYZ do numpy array
            point_cloud_np = np.array(point_cloud_data, dtype=np.float32).reshape((-1, 4))
            points = point_cloud_np[:, :3]  # XYZ
            colors = point_cloud_np[:, 3:]  # RGBA

            # Usunięcie wartości NaN lub Inf z kolorów
            valid_colors = np.isfinite(colors)
            points = points[valid_colors.all(axis=1)]
            colors = colors[valid_colors.all(axis=1)]

            # Normalizacja kolorów do zakresu [0, 1]
            colors = np.clip(colors / 255.0, 0, 1)

            # Tworzenie obiektu chmury punktów w Open3D
            o3d_pc = o3d.geometry.PointCloud()
            o3d_pc.points = o3d.utility.Vector3dVector(points)
            o3d_pc.colors = o3d.utility.Vector3dVector(colors)

            # Wyświetlenie chmury punktów
            o3d.visualization.draw_geometries([o3d_pc])

        else:
            print("Nie udało się pobrać chmury punktów.")
    else:
        print("Błąd podczas grabowania obrazu z kamery.")

def main():
    # Inicjalizacja kamery ZED
    zed = sl.Camera()

    # Parametry inicjalizacji kamery
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Rozdzielczość
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Tryb głębokości (najdokładniejszy)
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Jednostki w mm

    # Otwieranie kamery
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera.")
        exit(1)

    print("ZED camera opened successfully.")

    # Czekaj chwilę, aby kamera miała czas na inicjalizację
    time.sleep(1)

    # Pobranie i zapisanie chmury punktów
    filename = "point_cloud.ply"  # Możesz zmienić nazwę i format pliku
    save_and_display_point_cloud(zed, filename)

    # Zamykanie kamery
    zed.close()

if __name__ == "__main__":
    main()
