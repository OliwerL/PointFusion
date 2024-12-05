import time
import numpy as np
import open3d as o3d
import json


# Loads a point cloud from a PLY file
def load_point_cloud(ply_filename):
    return o3d.io.read_point_cloud(ply_filename)


# Applies a transformation to a point cloud using rotation matrix R and translation vector T
def apply_transformation(point_cloud, R, T):
    R = np.array(R)
    T = np.array(T) / 1000  # Convert translation from millimeters to meters
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = T.reshape(-1)
    point_cloud.transform(transformation_matrix)
    return point_cloud


# Merges multiple point clouds into one
def merge_point_clouds(clouds):
    merged_cloud = clouds[0]
    for cloud in clouds[1:]:
        merged_cloud += cloud
    return merged_cloud



# Visualizes a point cloud from a PLY file
def visualize_point_cloud(ply_file):
    # Load the point cloud from the PLY file
    pcd = o3d.io.read_point_cloud(ply_file)

    # Check if the point cloud was loaded correctly
    if pcd.is_empty():
        print("The point cloud is empty.")
        return

    # Display the point cloud
    o3d.visualization.draw_geometries([pcd])


# Loads calibration data from a .json file
def load_calibration_data(json_filename):
    with open(json_filename, 'r') as f:
        calibration_data = json.load(f)
    return calibration_data['R'], calibration_data['T']

# Main script
if __name__ == "__main__":
    # Start timing
    start_time = time.time()

    # Load calibration data
    R1, T1 = load_calibration_data('2_3_STEREO.json')
    R2, T2 = load_calibration_data('3_4_STEREO.json')

    # Load point clouds
    cloud1 = load_point_cloud('33_realsens_cleaned.ply')
    cloud2 = load_point_cloud('22_realsens_cleaned.ply')
    cloud3 = load_point_cloud('44_realsens_cleaned.ply')

    # Apply transformation to the second and third point clouds
    cloud2_transformed = apply_transformation(cloud2, R1, T1)
    cloud3_transformed = apply_transformation(cloud3, R2, T2)

    # Merge point clouds
    merged_cloud = merge_point_clouds([cloud1, cloud2_transformed, cloud3_transformed])

    # Save merged point cloud to a new PLY file
    output_filename = 'do_test.ply'
    o3d.io.write_point_cloud(output_filename, merged_cloud)
    end_time = time.time()

    print(f"Point clouds merged successfully. Output saved as '{output_filename}'")

    # Visualize the point cloud
    visualize_point_cloud(output_filename)

    # End timing
    print(f"Total execution time: {end_time - start_time:.6f} seconds")
