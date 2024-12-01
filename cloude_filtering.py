import numpy as np
import open3d as o3d


def filter_point_cloud(input_file, output_file, max_distance=3.0):
    """
    Filtruje punkty w chmurze punktów, usuwając te, które znajdują się w odległości większej niż max_distance od środka,
    jednocześnie zachowując kolory punktów (jeśli istnieją).

    Parameters:
    - input_file: Ścieżka do pliku wejściowego (np. .ply lub .pcd).
    - output_file: Ścieżka do pliku wyjściowego (np. .ply lub .pcd).
    - max_distance: Maksymalna dopuszczalna odległość od środka.
    """
    # Wczytaj chmurę punktów
    point_cloud = o3d.io.read_point_cloud(input_file)

    # Konwertuj chmurę punktów na numpy array
    points = np.asarray(point_cloud.points)

    # Sprawdź, czy chmura punktów zawiera informacje o kolorach
    if point_cloud.has_colors():
        colors = np.asarray(point_cloud.colors)
    else:
        colors = None

    # Oblicz środek masy chmury punktów
    center = points.mean(axis=0)

    # Oblicz odległość każdego punktu od środka
    distances = np.linalg.norm(points - center, axis=1)

    # Filtruj punkty w określonym promieniu
    filtered_indices = distances <= max_distance
    filtered_points = points[filtered_indices]

    # Jeżeli istnieją kolory, przefiltruj również kolory
    if colors is not None:
        filtered_colors = colors[filtered_indices]
    else:
        filtered_colors = None

    # Twórz nową chmurę punktów z przefiltrowanych danych
    filtered_cloud = o3d.geometry.PointCloud()
    filtered_cloud.points = o3d.utility.Vector3dVector(filtered_points)

    if filtered_colors is not None:
        filtered_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)

    # Zapisz przefiltrowaną chmurę punktów do pliku
    o3d.io.write_point_cloud(output_file, filtered_cloud)
    print(f"Przefiltrowana chmura punktów została zapisana do {output_file}")

if __name__ == "__main__":
    # Przykład użycia
    input_file = "te_dwie_co_chciales.ply"  # Zamień na ścieżkę do swojego pliku
    output_file = "te_dwie_co_chciales_test.ply"  # Ścieżka do zapisu przefiltrowanej chmury punktów
    filter_point_cloud(input_file, output_file, max_distance=1.7)
