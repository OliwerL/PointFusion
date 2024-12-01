import time
import numpy as np
import open3d as o3d
import json
import cv2

def load_point_cloud(ply_filename):
    return o3d.io.read_point_cloud(ply_filename)

def apply_transformation(point_cloud, R, T):
    R = np.array(R)
    T = np.array(T)/1000
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = T.reshape(-1)
    point_cloud.transform(transformation_matrix)
    return point_cloud

def merge_point_clouds(clouds):
    merged_cloud = clouds[0]
    for cloud in clouds[1:]:
        merged_cloud += cloud
    return merged_cloud

def visualize_point_cloud(ply_file):
    # Wczytaj chmurę punktów z pliku PLY
    pcd = o3d.io.read_point_cloud(ply_file)

    # Sprawdź, czy chmura punktów została poprawnie wczytana
    if pcd.is_empty():
        print("Chmura punktów jest pusta.")
        return

    # Wyświetl chmurę punktów
    o3d.visualization.draw_geometries([pcd])

def load_calibration_data(json_filename):
    """Wczytuje dane kalibracyjne z pliku .json."""
    with open(json_filename, 'r') as f:
        calibration_data = json.load(f)
    return calibration_data['R'], calibration_data['T']

# Main script
if __name__ == "__main__":
    # Start timing

    # Load calibration data
    with open('2_3_STEREO.json', 'r') as f:
        calibration_data1 = json.load(f)

    R1 = calibration_data1['R']
    T1 = calibration_data1['T']

    with open('3_4_STEREO.json', 'r') as f:
        calibration_data2 = json.load(f)

    R2 = calibration_data2['R']
    T2 = calibration_data2['T']
    start_time = time.time()
    # Load point clouds
    cloud1 = load_point_cloud('33_realsens_cleaned.ply')
    cloud2 = load_point_cloud('22_realsens_cleaned.ply')
    cloud3 = load_point_cloud('44_realsens_cleaned.ply')

    # Apply transformation to the second point cloud
    cloud2_transformed = apply_transformation(cloud2, R1, T1)
    cloud3_transformed = apply_transformation(cloud3, R2, T2)

    # Merge point clouds
    merged_cloud = merge_point_clouds([cloud1, cloud2_transformed, cloud3_transformed])

    # Save merged point cloud to a new PLY file
    o3d.io.write_point_cloud('do_test.ply', merged_cloud)
    end_time = time.time()

    print("Point clouds merged successfully. Output saved as 'merged_cloud.ply'")
    # Ścieżka do pliku PLY
    ply_file = 'do_test.ply'
    # Wizualizacja chmury punktów
    visualize_point_cloud(ply_file)

    # End timing

    print(f"Czas wykonania całej aplikacji: {end_time - start_time:.6f} sekund")
