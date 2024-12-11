import open3d as o3d


# Loads a point cloud from a .ply file.
def load_point_cloud(ply_filename):
    return o3d.io.read_point_cloud(ply_filename)


# Removes statistical outliers from a point cloud
def remove_isolated_points(point_cloud, nb_neighbors, std_ratio):
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    inlier_cloud = point_cloud.select_by_index(ind)
    return inlier_cloud


# Saves an Open3D point cloud to a .ply file
def save_point_cloud(ply_filename, point_cloud):
    o3d.io.write_point_cloud(ply_filename, point_cloud)


# Visualizes an Open3D point cloud
def visualize_point_cloud(point_cloud):
    o3d.visualization.draw_geometries([point_cloud])


if __name__ == "__main__":
    # Load point clouds from PLY files
    input_ply_file = '22_realsens.ply'
    cleaned_ply_file = '22_realsens_cleaned_1.ply'
    cloud = load_point_cloud(input_ply_file)

    input_ply_file1 = '33_realsens.ply'
    cleaned_ply_file1 = '33_realsens_cleaned_1.ply'
    cloud1 = load_point_cloud(input_ply_file1)

    input_ply_file2 = '44_realsens.ply'
    cleaned_ply_file2 = '44_realsens_cleaned_1.ply'
    cloud2 = load_point_cloud(input_ply_file2)

    # Check if the first point cloud is empty
    if cloud.is_empty():
        print("Point cloud is empty.")
    else:
        # Remove isolated points from each point cloud
        cleaned_cloud = remove_isolated_points(cloud, nb_neighbors=80, std_ratio=0.9)
        cleaned_cloud2 = remove_isolated_points(cloud1, nb_neighbors=80, std_ratio=0.9)
        cleaned_cloud3 = remove_isolated_points(cloud2, nb_neighbors=80, std_ratio=0.9)

        # Save the cleaned point clouds to new PLY files
        save_point_cloud(cleaned_ply_file, cleaned_cloud)
        save_point_cloud(cleaned_ply_file1, cleaned_cloud2)
        save_point_cloud(cleaned_ply_file2, cleaned_cloud3)

        # Visualize the cleaned point clouds
        visualize_point_cloud(cleaned_cloud)
        visualize_point_cloud(cleaned_cloud2)
        visualize_point_cloud(cleaned_cloud3)
