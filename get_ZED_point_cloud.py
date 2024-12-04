import pyzed.sl as sl
import open3d as o3d
import numpy as np
import time
import json


# Saves ZED camera intrinsic parameters to a JSON file.
def save_intrinsics_to_json(zed, filename):
    # Retrieve intrinsic parameters from the left camera
    camera_info = zed.get_camera_information()
    intrinsics = camera_info.calibration_parameters.left_cam

    # Prepare data for JSON
    intrinsics_data = {
        "fx": intrinsics.fx,
        "fy": intrinsics.fy,
        "cx": intrinsics.cx,
        "cy": intrinsics.cy,
        "coeffs": [
            intrinsics.k[0], intrinsics.k[1], intrinsics.k[2], intrinsics.k[3], intrinsics.k[4]
        ]
    }

    # Save intrinsic parameters to JSON file
    with open(filename, 'w') as json_file:
        json.dump(intrinsics_data, json_file, indent=4)

    print(f"Intrinsics saved to {filename}")

# Saves the point cloud captured by the ZED camera to a .ply file and displays it.
def save_and_display_point_cloud(zed, filename, intrinsics_filename):
    # Create a matrix to store the point cloud
    point_cloud = sl.Mat()

    # Capture a frame and retrieve the point cloud in XYZRGBA format
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        # Check if the point cloud contains data
        if point_cloud.is_init():
            num_points = point_cloud.get_width() * point_cloud.get_height()
            print(f"Point cloud contains {num_points} points.")

            # Save the point cloud to a file (e.g., .ply format)
            success = point_cloud.write(filename)
            if success == sl.ERROR_CODE.SUCCESS:
                print(f"Point cloud saved to {filename}")
            else:
                print("Failed to save point cloud.")

            # Convert ZED point cloud (sl.Mat) to NumPy array
            point_cloud_data = point_cloud.get_data()

            # Reshape data for Open3D
            point_cloud_np = np.array(point_cloud_data, dtype=np.float32).reshape((-1, 4))
            points = point_cloud_np[:, :3]  # XYZ coordinates
            colors = point_cloud_np[:, 3:]  # RGBA colors

            # Remove NaN or Inf values from colors
            valid_colors = np.isfinite(colors)
            points = points[valid_colors.all(axis=1)]
            colors = colors[valid_colors.all(axis=1)]

            # Normalize colors to [0,1]
            colors = np.clip(colors / 255.0, 0, 1)

            # Create Open3D point cloud object
            o3d_pc = o3d.geometry.PointCloud()
            o3d_pc.points = o3d.utility.Vector3dVector(points)
            o3d_pc.colors = o3d.utility.Vector3dVector(colors)

            # Visualize the point cloud
            o3d.visualization.draw_geometries([o3d_pc])

            # Save intrinsic parameters to JSON file
            save_intrinsics_to_json(zed, intrinsics_filename)

        else:
            print("Failed to retrieve point cloud data.")
    else:
        print("Error grabbing image from camera.")

# Initializes the ZED camera, captures the point cloud, and saves intrinsic parameters
def start_zed_cameras(filename, intrinsics_filename):
    # Initialize ZED camera
    zed = sl.Camera()

    # Set initialization parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Set resolution
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA          # Set depth mode to the most accurate
    init_params.coordinate_units = sl.UNIT.MILLIMETER    # Set units to millimeters

    # Open the camera
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera.")
        exit(1)


    # Allow the camera to initialize
    time.sleep(1)

    # Save and display the point cloud
    save_and_display_point_cloud(zed, filename, intrinsics_filename)

    # Close the camera after capturing
    zed.close()


if __name__ == "__main__":
    # File paths for saving point cloud and intrinsic parameters
    filename = "point_cloud.ply"           # Change the filename and format as needed
    intrinsics_filename = "intrinsics.json"  # File to save intrinsic parameters

    # Start the ZED camera capture and processing
    start_zed_cameras(filename, intrinsics_filename)
