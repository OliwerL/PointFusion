import open3d as o3d
import numpy as np
import json


# Loads a point cloud from a PLY file
def load_point_cloud(ply_filename):
    return o3d.io.read_point_cloud(ply_filename)


# Applies a transformation to a point cloud given rotation and translation matrices
def apply_transformation(point_cloud, R, T, invert=False):
    R = np.array(R)
    T = np.array(T) / -1000  # Scale the translation (from mm to meters)

    if invert:
        R = np.linalg.inv(R)  # Invert the rotation matrix
        T = -np.dot(R, T)  # Apply translation in the opposite direction

    # Create a 4x4 transformation matrix
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = T.reshape(-1)

    # Apply the transformation
    point_cloud.transform(transformation_matrix)
    return point_cloud


# Merges multiple point clouds into one
def merge_point_clouds(clouds):
    merged_cloud = o3d.geometry.PointCloud()
    for cloud in clouds:
        merged_cloud += cloud
    merged_cloud = merged_cloud.voxel_down_sample(voxel_size=0.005)  # Optional downsampling
    return merged_cloud


# Loads calibration data from a JSON file
def load_calibration_data(json_filename):
    with open(json_filename, 'r') as f:
        calibration_data = json.load(f)

    # Check data validity
    if not isinstance(calibration_data, dict):
        raise ValueError("Calibration file does not contain valid JSON data.")

    # Load R and T
    R = calibration_data.get('R')
    T = calibration_data.get('T')

    if R is None or T is None:
        raise ValueError("Missing 'R' or 'T' data in the calibration file.")

    return np.array(R), np.array(T)




# Computes the Fast Point Feature Histogram (FPFH) features for a point cloud
def compute_fpfh(pcd, voxel_size=0.05):



    down_pcd = pcd.voxel_down_sample(voxel_size)
    down_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down_pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    return down_pcd, fpfh


# Aligns and merges two point clouds using calibration data and various registration techniques
def algorithm(cloud1, cloud2, calibration_data_filename, output_filename, voxel_size=0.1):


    # Load calibration data
    R1, T1 = load_calibration_data(calibration_data_filename)

    # Apply transformation to the second point cloud
    cloud2_transformed = apply_transformation(cloud2, R1, T1, invert=True)

    # Estimate normals for both point clouds
    cloud1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    cloud2_transformed.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Calculate centers of the point clouds
    center1 = cloud1.get_center()
    center2 = cloud2_transformed.get_center()

    # Extracts the central region of a point cloud within a given radius from the center point
    def extract_center_region(point_cloud, center, radius):
        points = np.asarray(point_cloud.points)
        distances = np.linalg.norm(points - center, axis=1)
        mask = distances < radius
        center_region = point_cloud.select_by_index(np.where(mask)[0])
        return center_region

    # Extract central regions
    radius = 0.5  # Radius to define the central region
    cloud1_center_region = extract_center_region(cloud1, center1, radius)
    cloud2_center_region = extract_center_region(cloud2_transformed, center2, radius)

    # Translate clouds to their centers (optional, for better computation stability)
    cloud1_center_region.translate(-center1)
    cloud2_center_region.translate(-center2)
    cloud1.translate(-center1)
    cloud2_transformed.translate(-center2)

    # Compute FPFH features for the downsampled central regions
    down_cloud1_center, fpfh_cloud1_center = compute_fpfh(cloud1_center_region, voxel_size)
    down_cloud2_center, fpfh_cloud2_center = compute_fpfh(cloud2_center_region, voxel_size)

    # Initial alignment using RANSAC and FPFH
    distance_threshold = voxel_size * 3
    ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        down_cloud2_center, down_cloud1_center, fpfh_cloud2_center, fpfh_cloud1_center,
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

    # Apply the transformation from RANSAC
    cloud2_center_region.transform(ransac_result.transformation)
    cloud2_transformed.transform(ransac_result.transformation)

    # Refine alignment of central regions using ICP
    max_correspondence_distance = 0.3
    icp_result = o3d.pipelines.registration.registration_icp(
        cloud2_center_region, cloud1_center_region,
        max_correspondence_distance,
        init=np.identity(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    transformation = icp_result.transformation
    cloud2_center_region.transform(transformation)
    cloud2_transformed.transform(transformation)

    # Color ICP refinement
    max_correspondence_distance = 0.1  # Maximum correspondence distance
    lambda_geometric = 0.968
    icp_colored_result = o3d.pipelines.registration.registration_colored_icp(
        cloud2_transformed, cloud1,
        max_correspondence_distance,
        init=icp_result.transformation,  # Result from previous ICP
        estimation_method=o3d.pipelines.registration.TransformationEstimationForColoredICP(lambda_geometric),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6,
            relative_rmse=1e-6,
            max_iteration=50
        )
    )
    # Apply the transformation to the entire point cloud
    cloud2_transformed.transform(icp_colored_result.transformation)

    # Merge the point clouds
    resulting_cloud = merge_point_clouds([cloud1, cloud2_transformed])

    # Translate the resulting cloud back to the original center
    resulting_cloud.translate(center1)
    # Save the resulting point cloud
    o3d.io.write_point_cloud(output_filename, resulting_cloud)

if __name__ == "__main__":
    # Path to the calibration data file
    calibration_data_filename = 'zedy_calibration/stereo_calibration_3.json'
    # Load point clouds
    cloud1 = load_point_cloud('3_po.ply')
    cloud2 = load_point_cloud('4_po.ply')
    output_filename = "te_trzy_co_chciales.ply"

    # Run the algorithm
    algorithm(cloud1, cloud2, calibration_data_filename, output_filename)