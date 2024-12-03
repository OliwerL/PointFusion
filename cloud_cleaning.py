import open3d as o3d

def load_point_cloud(ply_filename):
    return o3d.io.read_point_cloud(ply_filename)

def remove_isolated_points(point_cloud, nb_neighbors, std_ratio):
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    inlier_cloud = point_cloud.select_by_index(ind)
    return inlier_cloud

def save_point_cloud(ply_filename, point_cloud):
    o3d.io.write_point_cloud(ply_filename, point_cloud)

def visualize_point_cloud(point_cloud):
    o3d.visualization.draw_geometries([point_cloud])

if __name__ == "__main__":
    # Load point cloud from PLY file
    input_ply_file = '22_realsens.ply'
    cleaned_ply_file = '22_realsens_cleaned_1.ply'
    cloud = load_point_cloud(input_ply_file)
    input_ply_file1 = '33_realsens.ply'
    cleaned_ply_file1 = '33_realsens_cleaned_1.ply'
    cloud1 = load_point_cloud(input_ply_file1)
    input_ply_file2 = '44_realsens.ply'
    cleaned_ply_file2 = '44_realsens_cleaned_1.ply'
    cloud2 = load_point_cloud(input_ply_file2)

    if cloud.is_empty():
        print("Chmura punktów jest pusta.")
    else:
        # Remove isolated points
        cleaned_cloud = remove_isolated_points(cloud, nb_neighbors=80, std_ratio=0.9)
        cleaned_cloud2 = remove_isolated_points(cloud1, nb_neighbors=80, std_ratio=0.9)
        cleaned_cloud3 = remove_isolated_points(cloud2, nb_neighbors=80, std_ratio=0.9)

        # Save cleaned point cloud to a new PLY file
        save_point_cloud(cleaned_ply_file, cleaned_cloud)
        save_point_cloud(cleaned_ply_file1, cleaned_cloud2)
        save_point_cloud(cleaned_ply_file2, cleaned_cloud3)

        # Visualize the cleaned point cloud
        visualize_point_cloud(cleaned_cloud)
        visualize_point_cloud(cleaned_cloud2)
        visualize_point_cloud(cleaned_cloud3)
