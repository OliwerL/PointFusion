import open3d as o3d


# Visualizes a point cloud from a PLY file
def visualize_point_cloud(ply_file):
    pcd = o3d.io.read_point_cloud(ply_file)

    # Check if the point cloud was loaded correctly
    if pcd.is_empty():
        print("The point cloud is empty.")
        return

    # Display the point cloud
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    # Path to the PLY files
    # ply_file = 'prawyPLY.ply'
    ply_file = 'te_dwie_co_chciales.ply'
    ply_file1 = 'te_dwie_co_chciales_test.ply'
    # ply_file2 = '44_realsens.ply'

    # Load and visualize the first point cloud
    point_cloud = o3d.io.read_point_cloud(ply_file)
    visualize_point_cloud(ply_file)

    # Load and visualize the second point cloud
    point_cloud1 = o3d.io.read_point_cloud(ply_file1)
    visualize_point_cloud(ply_file1)

    # Load and visualize the third point cloud (commented out)
    # point_cloud2 = o3d.io.read_point_cloud(ply_file2)
    # visualize_point_cloud(ply_file2)

    # Print the number of points in the original point clouds
    print(f"Number of points in the original point cloud: {len(point_cloud.points)}")
    print(f"Number of points in the original point cloud: {len(point_cloud1.points)}")
    # print(f"Number of points in the original point cloud: {len(point_cloud2.points)}")
