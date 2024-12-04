import pyrealsense2 as rs
import numpy as np
import open3d as o3d

# Extracts color and depth frames from a RealSense .bag file.
def extract_point_cloud_from_bag(path_bag):
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, path_bag)

    # Start streaming
    pipeline.start(config)

    # Create an align object to align depth frames to color frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Skipping initial frames to allow for auto-exposure stabilization
    for _ in range(30):
        pipeline.wait_for_frames()

    # Get aligned frames
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Extract aligned depth and color frames
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # Stop the pipeline
    pipeline.stop()

    return color_frame, depth_frame

# Converts RealSense color and depth frames to an Open3D point cloud
def convert_to_open3d_point_cloud(color_frame, depth_frame):


    # Convert images to NumPy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # Convert depth image to point cloud
    pc = rs.pointcloud()
    points = pc.calculate(depth_frame)
    vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)  # XYZ coordinates

    # Create Open3D point cloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(vtx)

    # Extract color information and assign to point cloud
    rgb = color_image.reshape(-1, 3) / 255.0  # Normalize RGB values to [0,1]
    point_cloud.colors = o3d.utility.Vector3dVector(rgb)

    return point_cloud

# Saves an Open3D point cloud to a .ply file.
def save_point_cloud_to_ply(point_cloud, ply_filename):
    o3d.io.write_point_cloud(ply_filename, point_cloud)


if __name__ == "__main__":
    # Paths to .bag files and corresponding output .ply files
    bag_file1 = 'chujciwdupelisek/20241120_151630.bag'
    bag_file2 = 'chujciwdupelisek/20241120_151635.bag'
    bag_file3 = 'chujciwdupelisek/20241120_151744.bag'
    ply_file1 = '333_realsens.ply'
    ply_file2 = '444_realsens.ply'
    ply_file3 = '44_realsens.ply'

    # Extract and convert point clouds from each .bag file
    color_frame1, depth_frame1 = extract_point_cloud_from_bag(bag_file1)
    color_frame2, depth_frame2 = extract_point_cloud_from_bag(bag_file2)
    color_frame3, depth_frame3 = extract_point_cloud_from_bag(bag_file3)

    # Convert frames to Open3D point clouds
    point_cloud1 = convert_to_open3d_point_cloud(color_frame1, depth_frame1)
    point_cloud2 = convert_to_open3d_point_cloud(color_frame2, depth_frame2)
    point_cloud3 = convert_to_open3d_point_cloud(color_frame3, depth_frame3)

    # Save the point clouds to .ply files
    save_point_cloud_to_ply(point_cloud1, ply_file1)
    save_point_cloud_to_ply(point_cloud2, ply_file2)
    save_point_cloud_to_ply(point_cloud3, ply_file3)

    print(f"Point clouds saved to {ply_file1}, {ply_file2}, and {ply_file3}")
