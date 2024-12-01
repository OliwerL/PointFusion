import numpy as np
import open3d as o3d
import json
from granularity import zmien_granulacje_chmury_punktow

def load_point_cloud(ply_filename):
    return o3d.io.read_point_cloud(ply_filename)


def apply_transformation(point_cloud, R, T, invert=False):
    R = np.array(R)
    T = np.array(T) / -1000  # Przeskalowanie przesunięcia (z mm na metry)

    if invert:
        R = np.linalg.inv(R)  # Odwrócenie macierzy obrotu
        T = -np.dot(R, T)  # Przesunięcie w odwrotnym kierunku

    # Tworzenie macierzy transformacji 4x4
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = T.reshape(-1)

    # Zastosowanie transformacji
    point_cloud.transform(transformation_matrix)
    return point_cloud


def merge_point_clouds(clouds):
    merged_cloud = clouds[0]
    for cloud in clouds[1:]:
        merged_cloud += cloud
    return merged_cloud


def load_calibration_data(json_filename):
    """Wczytaj dane kalibracji z pliku JSON."""
    with open(json_filename, 'r') as f:
        calibration_data = json.load(f)

    # Sprawdzenie poprawności danych
    if not isinstance(calibration_data, dict):
        raise ValueError("Plik kalibracyjny nie zawiera poprawnych danych JSON.")

    # Wczytanie R i T
    R = calibration_data.get('R')
    T = calibration_data.get('T')

    if R is None or T is None:
        raise ValueError("Brakuje danych 'R' lub 'T' w pliku kalibracyjnym.")

    return np.array(R), np.array(T)

def algorithm(cloud1, cloud2, calibration_data1, output_filename, voxel_size=0.1):
    # Load calibration data

    R1 ,T1 = load_calibration_data(calibration_data1)

    # Apply transformation to the second point cloud
    cloud2_transformed = apply_transformation(cloud2, R1, T1, True)

    chmura1 = cloud1
    chmura2 = cloud2_transformed
    chmura1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    chmura2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    def extract_center_region(point_cloud, center, radius):
        points = np.asarray(point_cloud.points)
        distances = np.linalg.norm(points - center, axis=1)
        mask = distances < radius
        center_region = point_cloud.select_by_index(np.where(mask)[0])
        return center_region

    # Oblicz środki chmur
    center1 = chmura1.get_center()
    center2 = chmura2.get_center()

    # Wyodrębnij centralne regiony
    radius = 0.5  # Promień w jednostkach chmury punktów
    chmura1_center = extract_center_region(chmura1, center1, radius)
    chmura2_center = extract_center_region(chmura2, center2, radius)

    # Przesuń chmury do wspólnego środka (opcjonalne, dla lepszej stabilności obliczeń)
    chmura1_center.translate(-center1)
    chmura2_center.translate(-center2)
    chmura1.translate(-center1)
    chmura2.translate(-center2)
    # Funkcja do obliczania cech FPFH

    def compute_fpfh(pcd, voxel_size=0.05):
        down_pcd = pcd.voxel_down_sample(voxel_size)
        down_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            down_pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
        return down_pcd, fpfh

    down_chmura1, fpfh_chmura1 = compute_fpfh(chmura1_center, voxel_size)
    down_chmura2, fpfh_chmura2 = compute_fpfh(chmura2_center, voxel_size)

    # Wstępne dopasowanie z RANSAC i FPFH
    distance_threshold = voxel_size * 3
    wynik_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        down_chmura2, down_chmura1, fpfh_chmura2, fpfh_chmura1,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    )

    # Zastosowanie transformacji z RANSAC
    chmura2_center.transform(wynik_ransac.transformation)
    chmura2.transform(wynik_ransac.transformation)

    # Dopasowanie centralnych regionów za pomocą ICP
    max_correspondence_distance = 0.3
    icp_result = o3d.pipelines.registration.registration_icp(
        chmura2_center, chmura1_center,
        max_correspondence_distance,
        init=np.identity(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    transformacja = icp_result.transformation
    chmura2_center.transform(transformacja)
    chmura2.transform(transformacja)

    max_correspondence_distance = 0.1  # Maksymalna odległość korespondencji
    lambda_geometric = 0.968
    # Dopasowanie z kolorem
    icp_colored_result = o3d.pipelines.registration.registration_colored_icp(
        chmura2, chmura1,
        max_correspondence_distance,
        init=icp_result.transformation,  # Wynik wstępnego ICP
        estimation_method=o3d.pipelines.registration.TransformationEstimationForColoredICP(lambda_geometric),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6,
                                                                   max_iteration=50)
    )
    # Zastosuj transformację do całej chmury punktów
    chmura2.transform(icp_colored_result.transformation)
    resulting_cloud = merge_point_clouds([chmura1, chmura2])

    resulting_cloud.translate(center1)
    o3d.io.write_point_cloud(output_filename, resulting_cloud)


if __name__ == "__main__":
    with open('zedy_calibration/stereo_calibration_3.json', 'r') as f:
        calibration_data1 = json.load(f)
    # Load point clouds
    cloud1 = load_point_cloud('3_po.ply')
    cloud2 = load_point_cloud('4_po.ply')
    output_filename = "te_trzy_co_chciales.ply"

    algorithm(cloud1, cloud2, calibration_data1, output_filename)