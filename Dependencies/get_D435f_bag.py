import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import json
import time


# Captures point clouds and intrinsic parameters from RealSense cameras,
# saves the point cloud to a PLY file, and intrinsic parameters to a JSON file.

def capture_and_process_intrinsics(output_ply_filename, output_json_filename, duration=10):

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable RGB and Depth streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # RGB stream
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)    # Depth stream

    # Start the pipeline
    profile = pipeline.start(config)

    # Get camera streams
    color_stream = profile.get_stream(rs.stream.color)
    depth_stream = profile.get_stream(rs.stream.depth)

    # Get intrinsic parameters for both color and depth streams
    color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
    depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

    # Prepare intrinsic parameters data
    intrinsics_data = {
        "color": {
            "width": color_intrinsics.width,
            "height": color_intrinsics.height,
            "fx": color_intrinsics.fx,
            "fy": color_intrinsics.fy,
            "cx": color_intrinsics.ppx,
            "cy": color_intrinsics.ppy,
            "coeffs": color_intrinsics.coeffs
        },
        "depth": {
            "width": depth_intrinsics.width,
            "height": depth_intrinsics.height,
            "fx": depth_intrinsics.fx,
            "fy": depth_intrinsics.fy,
            "cx": depth_intrinsics.ppx,
            "cy": depth_intrinsics.ppy,
            "coeffs": depth_intrinsics.coeffs
        }
    }

    # Save intrinsic parameters to JSON file
    with open(output_json_filename, 'w') as json_file:
        json.dump(intrinsics_data, json_file, indent=4)

    print(f"Intrinsics saved to {output_json_filename}")

    # Initialize point cloud object
    pc = rs.pointcloud()
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Start capturing frames for the specified duration
    start_time = time.time()

    try:
        while True:
            # Check if the specified duration has elapsed
            elapsed_time = time.time() - start_time
            if elapsed_time > duration:
                print(f"Recording time elapsed ({duration} seconds). Stopped data collection.")
                break

            # Wait for the next set of frames
            frames = pipeline.wait_for_frames()

            # Align the depth frame to the color frame
            aligned_frames = align.process(frames)

            # Extract aligned depth and color frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Map depth data to the color frame
            pc.map_to(color_frame)
            pointcloud = pc.calculate(depth_frame)

            # Convert point cloud data to NumPy arrays
            vertices = np.asanyarray(pointcloud.get_vertices())  # XYZ coordinates
            colors = np.asanyarray(color_frame.get_data())       # RGB colors

            # Create Open3D point cloud
            points = np.array([[point[0], point[1], point[2]] for point in vertices])  # XYZ
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # Normalize and assign colors
            rgb = colors.reshape(-1, 3) / 255.0  # Normalize RGB to [0,1]
            pcd.colors = o3d.utility.Vector3dVector(rgb)

            # Save the point cloud to a PLY file
            o3d.io.write_point_cloud(output_ply_filename, pcd)
            print(f"Point cloud saved to {output_ply_filename}")

    finally:
        # Stop the pipeline after capturing
        pipeline.stop()


if __name__ == "__main__":
    # File paths for saving point cloud and intrinsic parameters
    output_ply_filename = "point_cloud.ply"
    output_json_filename = "intrinsics.json"
    duration = 10  # Duration in seconds to capture data

    # Execute the capture and processing function
    capture_and_process_intrinsics(output_ply_filename, output_json_filename, duration)
