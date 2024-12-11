import numpy as np
import open3d as o3d


# Filters points in the point cloud, removing those that are more than max_distance from the center,
# while keeping the colors of the points (if any).
def filter_point_cloud(input_file, output_file, max_distance=3.0):
    # Load the point cloud from the input file
    point_cloud = o3d.io.read_point_cloud(input_file)

    # Convert the point cloud to a NumPy array
    points = np.asarray(point_cloud.points)

    # Check if the point cloud contains color information
    if point_cloud.has_colors():
        colors = np.asarray(point_cloud.colors)
    else:
        colors = None

    # Calculate the centroid of the point cloud
    center = points.mean(axis=0)

    # Calculate the distance of each point from the centroid
    distances = np.linalg.norm(points - center, axis=1)

    # Filter points within the specified distance
    filtered_indices = distances <= max_distance
    filtered_points = points[filtered_indices]

    # If colors exist, filter them as well
    if colors is not None:
        filtered_colors = colors[filtered_indices]
    else:
        filtered_colors = None

    # Create a new point cloud from the filtered data
    filtered_cloud = o3d.geometry.PointCloud()
    filtered_cloud.points = o3d.utility.Vector3dVector(filtered_points)

    if filtered_colors is not None:
        filtered_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)

    # Save the filtered point cloud to the output file
    o3d.io.write_point_cloud(output_file, filtered_cloud)
    print(f"Filtered point cloud has been saved to {output_file}")

if __name__ == "__main__":
    # Example usage
    input_file = "te_dwie_co_chciales.ply"  # Replace with the path to your input file
    output_file = "te_dwie_co_chciales_test.ply"  # Path to save the filtered point cloud
    filter_point_cloud(input_file, output_file, max_distance=1.7)
