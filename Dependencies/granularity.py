import open3d as o3d

# Changes the granularity of a point cloud by applying voxel downsampling
def change_granularity_of_point_cloud(input_filename, voxel_size, output_filename):

    # Load the point cloud from the .ply file
    point_cloud = o3d.io.read_point_cloud(input_filename)
    print(f"Number of points before downsampling: {len(point_cloud.points)}")

    # Apply Voxel Downsampling to reduce the number of points
    downsampled_cloud = point_cloud.voxel_down_sample(voxel_size=voxel_size)
    print(f"Number of points after downsampling: {len(downsampled_cloud.points)}")

    # Save the downsampled point cloud to a new .ply file
    o3d.io.write_point_cloud(output_filename, downsampled_cloud)
    print(f"Downsampled point cloud saved as {output_filename}")


if __name__ == "__main__":
    # Example usage
    input_filename = "srodek.ply"
    voxel_size = 0.01  # Set voxel size as needed
    output_filename = "srodek_original.ply"

    change_granularity_of_point_cloud(input_filename, voxel_size, output_filename)
