import sys
import os
import threading
import json
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget, QPushButton, QVBoxLayout,
    QHBoxLayout, QLabel, QFileDialog, QMessageBox, QInputDialog, QStatusBar
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QWindow
from PySide6.QtGui import QFont
import win32gui
import open3d as o3d  # Make sure to import open3d if used
from Dependencies.cloud_filtering import filter_point_cloud

# Import functions from files
from Dependencies.bag_to_ply import extract_point_cloud_from_bag, convert_to_open3d_point_cloud, save_point_cloud_to_ply
from Dependencies.photo import capture_photos
from Dependencies.multiphoto import multi_camera
from Dependencies.parameters import get_intrinsics_from_bag
from Dependencies.merge_clouds import load_point_cloud, apply_transformation, merge_point_clouds, load_calibration_data
from Dependencies.granularity import change_granularity_of_point_cloud
from Dependencies.stereo import load_intrinsics, stereo_calibrate, calculate_translation_distance, calculate_rotation_angle
from Dependencies.stereo_zed import stereo_calibrate, calculate_translation_distance, calculate_rotation_angle, load_intrinsics_from_conf
from Dependencies.zed_sn import capture_zed_camera, open_zed_camera
from Dependencies.merge_point_clouds_icp import algorithm
from Dependencies.cloud_cleaning import remove_isolated_points
from Dependencies.get_D435f_bag import capture_and_process_intrinsics
from Dependencies.get_ZED_point_cloud import start_zed_cameras

def resource_path(relative_path):
    try:
        # If the app is run as a bundle
        base_path = sys._MEIPASS
    except Exception:
        # If the app is run as a script
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)


class PointCloudApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Point Cloud GUI")
        # Variable storing the camera selection (0 = ZED, 1 = RealSense)
        self.camera_type = self.choose_camera_type()

        # If the user canceled the selection, close the application
        if self.camera_type is None:
            sys.exit()

        self.setWindowTitle("Point Cloud GUI")
        self.setGeometry(100, 100, 1600, 800)

        # Configure the main layout
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # Create tabs and buttons on the left side
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
                    QTabBar::tab {
                        font-size: 9pt;
                    }
                    QTabBar {
                        qproperty-expanding: 1;  /* Forces tabs to stretch to full width */
                    }
                """)
        self.create_tabs()
        main_layout.addWidget(self.tabs, stretch=1)

        # Configure Open3D on the right side
        filtered_file_path = resource_path("filtered_output_change.ply")
        self.pcd = o3d.io.read_point_cloud(filtered_file_path)
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.add_geometry(self.pcd)

        # Get the Open3D window handle for integration with PySide6
        hwnd = win32gui.FindWindowEx(0, 0, None, "Open3D")
        self.o3d_window = QWindow.fromWinId(hwnd)
        self.o3d_container = self.createWindowContainer(self.o3d_window, main_widget)
        main_layout.addWidget(self.o3d_container, stretch=2)

        # Timer to refresh Open3D visualization
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_vis)
        self.timer.start(16)  # Refresh every ~16 ms (approximately 60 FPS)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")

    def choose_camera_type(self):
        items = ["ZED", "RealSense"]
        camera_choice, ok = QInputDialog.getItem(
            self,
            "Camera Selection",
            "Choose the type of camera you want to use:",
            items,
            0,
            False,
        )
        if ok:
            return 0 if camera_choice == "ZED" else 1
        else:
            return None

    #  all tabs to QTabWidget
    def create_tabs(self):

        # Initialize individual tabs
        self.processing_tab = QWidget()
        self.visualization_tab = QWidget()
        self.camera_tab = QWidget()
        self.segmentation_tab = QWidget()
        self.calibration_tab = QWidget()

        # Set up tabs
        self.create_processing_tab()
        self.create_visualization_tab()
        self.create_camera_tab()
        self.create_calibration_tab()

        # Add tabs to QTabWidget
        self.tabs.addTab(self.processing_tab, "Processing")
        self.tabs.addTab(self.visualization_tab, "Visualization")
        self.tabs.addTab(self.camera_tab, "Cameras")
        self.tabs.addTab(self.calibration_tab, "Calibration")

    def create_processing_tab(self):
        layout = QVBoxLayout()
        button_font = QFont('Helvetica', 11)

        if self.camera_type == 1:  # RealSense
            self.bag_to_ply_button = QPushButton("Convert BAG to PLY")
            self.bag_to_ply_button.setFont(button_font)
            self.bag_to_ply_button.clicked.connect(self.process_bag_to_ply)
            layout.addWidget(self.bag_to_ply_button)

            self.intrinsics_button = QPushButton("Save Intrinsic Parameters")
            self.intrinsics_button.setFont(button_font)
            self.intrinsics_button.clicked.connect(self.save_intrinsics)
            layout.addWidget(self.intrinsics_button)

            self.merge_clouds_button = QPushButton("Merge Point Clouds")
            self.merge_clouds_button.setFont(button_font)
            self.merge_clouds_button.clicked.connect(self.start_merge_point_clouds)
            layout.addWidget(self.merge_clouds_button)

            self.download_param_realsense = QPushButton("Download Cloud with Parameters")
            self.download_param_realsense.setFont(button_font)
            self.download_param_realsense.clicked.connect(self.get_cloud_and_intristics_realsense)
            layout.addWidget(self.download_param_realsense)

        self.change_granularity_button = QPushButton("Change Point Cloud Granularity")
        self.change_granularity_button.setFont(button_font)
        self.change_granularity_button.clicked.connect(self.start_change_granularity)
        layout.addWidget(self.change_granularity_button)

        if self.camera_type == 0:  # ZED
            self.merge_clouds_icp_button = QPushButton("Merge Point Clouds with ICP")
            self.merge_clouds_icp_button.setFont(button_font)
            self.merge_clouds_icp_button.clicked.connect(self.start_merge_point_clouds_with_icp)
            layout.addWidget(self.merge_clouds_icp_button)

            self.download_param_zed = QPushButton("Download Cloud with Parameters")
            self.download_param_zed.setFont(button_font)
            self.download_param_zed.clicked.connect(self.get_cloud_and_intriscs_zed)
            layout.addWidget(self.download_param_zed)

        self.processing_tab.setLayout(layout)

    def create_visualization_tab(self):
        layout = QVBoxLayout()
        button_font = QFont('Helvetica', 11)

        self.show_cloud_button = QPushButton("Show Point Cloud")
        self.show_cloud_button.setFont(button_font)
        self.show_cloud_button.clicked.connect(self.show_point_cloud)
        layout.addWidget(self.show_cloud_button)

        # Button to filter the cloud
        self.filter_cloud_button = QPushButton("Crop Cloud")
        self.filter_cloud_button.setFont(button_font)
        self.filter_cloud_button.clicked.connect(self.filter_and_show_point_cloud)
        layout.addWidget(self.filter_cloud_button)

        self.remove_outliers_button = QPushButton("Clean Cloud")
        self.remove_outliers_button.setFont(button_font)
        self.remove_outliers_button.clicked.connect(self.remove_outlisers_from_cloude)
        layout.addWidget(self.remove_outliers_button)

        self.visualization_tab.setLayout(layout)

    def create_camera_tab(self):
        layout = QVBoxLayout()
        button_font = QFont('Helvetica', 11)

        if self.camera_type == 1:  # RealSense
            self.camera_button = QPushButton("Start Single Camera")
            self.camera_button.setFont(button_font)
            self.camera_button.clicked.connect(self.start_camera)
            layout.addWidget(self.camera_button)

            self.multi_camera_button = QPushButton("Start Dual Cameras")
            self.multi_camera_button.setFont(button_font)
            self.multi_camera_button.clicked.connect(self.start_multi_camera_capture)
            layout.addWidget(self.multi_camera_button)

        if self.camera_type == 0:  # ZED


            self.dual_zed_button = QPushButton("Start Dual ZED Cameras")
            self.dual_zed_button.setFont(button_font)
            self.dual_zed_button.clicked.connect(self.start_dual_zed_cameras)
            layout.addWidget(self.dual_zed_button)

        self.camera_tab.setLayout(layout)

    def create_calibration_tab(self):
        layout = QVBoxLayout()
        button_font = QFont('Helvetica', 11)

        if self.camera_type == 1:  # RealSense
            self.stereo_calibration_button = QPushButton("Stereo Calibration")
            self.stereo_calibration_button.setFont(button_font)
            self.stereo_calibration_button.clicked.connect(self.start_stereo_calibration)
            layout.addWidget(self.stereo_calibration_button)
        if self.camera_type == 0:  # ZED
            self.zed_calibration_button = QPushButton("Stereo Calibration for ZED")
            self.zed_calibration_button.setFont(button_font)
            self.zed_calibration_button.clicked.connect(self.start_zed_stereo_calibration)
            layout.addWidget(self.zed_calibration_button)

        self.calibration_tab.setLayout(layout)

    # Refreshes the Open3D visualization
    def update_vis(self):

        self.vis.poll_events()
        self.vis.update_renderer()


    # Updates the status bar with the given message
    def update_status(self, message):

        self.status_bar.showMessage(message)
        QApplication.processEvents()

    def process_bag_to_ply(self):
        # Select .bag file
        bag_path, _ = QFileDialog.getOpenFileName(self, "Select .bag File", "", "BAG Files (*.bag)",
                                                  options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if bag_path:
            # User selects the name of the .ply file to save
            ply_filename, _ = QFileDialog.getSaveFileName(self, "Save .ply File", "", "PLY Files (*.ply)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
            if ply_filename:
                if not ply_filename.lower().endswith(".ply"):
                    ply_filename += ".ply"
                # Update status bar
                self.update_status("Processing .bag file to .ply...")
                # Call function from bag_to_ply.py
                color_frame, depth_frame = extract_point_cloud_from_bag(bag_path)
                self.point_cloud = convert_to_open3d_point_cloud(color_frame, depth_frame)
                save_point_cloud_to_ply(self.point_cloud, ply_filename)
                self.update_status(f"Point cloud saved to file: {ply_filename}")

                # Display success message
                QMessageBox.information(self, "Success", f"Point cloud saved to file: {ply_filename}")

    # Allows the user to select a .ply file and displays it in the Open3D window.
    def show_point_cloud(self):

        ply_path, _ = QFileDialog.getOpenFileName(self, "Select .ply File", "", "PLY Files (*.ply)",
                                                  options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if ply_path:
            self.update_status("Loading selected point cloud...")
            new_pcd = o3d.io.read_point_cloud(ply_path)
            self.vis.clear_geometries()  # Remove the previous point cloud
            self.vis.add_geometry(new_pcd)  # Add the new point cloud
            self.pcd = new_pcd  # Update self.pcd
            self.update_status(f"Displayed point cloud: {ply_path}")
        else:
            QMessageBox.critical(self, "Error", "No .ply file selected")

    # Starts capturing photos from a single camera.
    def start_camera(self):

        QMessageBox.information(self, "Information", "Press 's' to save a photo\nPress 'ESC' to exit camera preview")

        camera_index, ok = QInputDialog.getInt(self, "Camera Index", "Enter camera index (e.g., 0, 1, 2):")
        if not ok:
            return

        base_name, ok = QInputDialog.getText(self, "Folder Name", "Enter folder name for photos:")
        if not ok or not base_name:
            return

        output_folder = f"{base_name}_PHOTOS"
        os.makedirs(output_folder, exist_ok=True)

        self.update_status("Starting camera...")

        threading.Thread(target=capture_photos, args=(camera_index, output_folder)).start()

    # Saves the intrinsic parameters from a .bag file to a JSON file.
    def save_intrinsics(self):

        bag_path, _ = QFileDialog.getOpenFileName(self, "Select .bag File", "", "BAG Files (*.bag)",
                                                  options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if bag_path:
            json_filename, _ = QFileDialog.getSaveFileName(self, "Save .json File", "", "JSON Files (*.json)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
            if json_filename:
                if not json_filename.lower().endswith(".json"):
                    json_filename += ".json"
                self.update_status("Saving intrinsic parameters...")
                get_intrinsics_from_bag(bag_path, json_filename)
                self.update_status(f"Intrinsic parameters saved to file: {json_filename}")
                QMessageBox.information(self, "Success", f"Intrinsic parameters saved to file: {json_filename}")

    # Starts capturing photos from multiple cameras.
    def start_multi_camera_capture(self):

        indices_input, ok = QInputDialog.getText(self, "Camera Indices", "Enter camera indices separated by commas (e.g., 0,1,2):")
        if not ok or not indices_input:
            return

        try:
            camera_indices = [int(idx.strip()) for idx in indices_input.split(",")]
        except ValueError:
            QMessageBox.critical(self, "Error", "Invalid camera indices format.")
            return

        base_name, ok = QInputDialog.getText(self, "Folder Name", "Enter folder name for multi-camera capture photos:")
        if not ok or not base_name:
            return

        output_folder = f"{base_name}_MultiCamera_PHOTOS"
        os.makedirs(output_folder, exist_ok=True)

        self.update_status("Starting multi-camera capture...")

        threading.Thread(target=multi_camera, args=(camera_indices, output_folder)).start()

    # Merges multiple point clouds based on user input.
    def start_merge_point_clouds(self):

        num_clouds, ok = QInputDialog.getInt(self, "Number of Clouds", "Enter the number of point clouds to merge:", 2, 2, 100)

        if not ok:
            return

        ply_filenames = []
        for i in range(num_clouds):
            ply_filename, _ = QFileDialog.getOpenFileName(self, f"Select .ply File for Cloud {i + 1}", "",
                                                          "PLY Files (*.ply)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
            if not ply_filename:
                QMessageBox.critical(self, "Error", "No PLY file selected for one of the clouds.")
                return
            ply_filenames.append(ply_filename)

        calibration_files = []
        for i in range(1, num_clouds):
            json_filename, _ = QFileDialog.getOpenFileName(
                self,
                f"Select calibration .json file for transforming Cloud {i + 1} to Cloud 1",
                "",
                "JSON Files (*.json)",
                options=QFileDialog.Option(QFileDialog.DontUseNativeDialog)
            )
            if not json_filename:
                QMessageBox.critical(self, "Error", "No JSON file selected for one of the transformations.")
                return
            calibration_files.append(json_filename)

        clouds = []
        # Ask for voxel size for each cloud
        voxel_size, ok = QInputDialog.getDouble(
            self,
            "Voxel Size",
            "Enter voxel size for the clouds (e.g., 0.01, range 0.001 - 0.1):",
            0.01, 0.001, 0.1, 3
        )
        if not ok:
            return

        for i, ply_filename in enumerate(ply_filenames):
            point_cloud = load_point_cloud(ply_filename)

            try:
                point_cloud = point_cloud.voxel_down_sample(voxel_size)  # Change granularity for this cloud
                self.update_status(f"Granularity for cloud {i + 1} set to {voxel_size}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to change granularity for cloud {i + 1}: {e}")
                return

            if i == 0:
                clouds.append(point_cloud)
            else:
                R, T = load_calibration_data(calibration_files[i - 1])
                transformed_cloud = apply_transformation(point_cloud, R, T)
                clouds.append(transformed_cloud)

        # After changing granularity, merge the clouds
        merged_cloud = merge_point_clouds(clouds)

        merged_filename, _ = QFileDialog.getSaveFileName(self, "Save Merged Point Cloud", "",
                                                         "PLY Files (*.ply)",
                                                         options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not merged_filename.lower().endswith(".ply"):
            merged_filename += ".ply"
        if not merged_filename:
            return

        o3d.io.write_point_cloud(merged_filename, merged_cloud)
        self.update_status(f"Point clouds merged and saved to file: {merged_filename}")
        QMessageBox.information(self, "Success", f"Point clouds merged and saved to file: {merged_filename}")

        # Update visualization
        self.update_status("Updating visualization...")
        self.vis.clear_geometries()  # Remove old point cloud from the visualizer
        self.vis.add_geometry(merged_cloud)  # Add new merged point cloud
        self.pcd = merged_cloud  # Update reference to the point cloud
        self.update_status("Ready")

    # Merges two point clouds using the ICP algorithm.
    def start_merge_point_clouds_with_icp(self):

        # Select point cloud files
        cloud1_file, _ = QFileDialog.getOpenFileName(self, "Select First Point Cloud (PLY)", "",
                                                     "PLY Files (*.ply)", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not cloud1_file:
            QMessageBox.critical(self, "Error", "No first point cloud selected.")
            return

        cloud2_file, _ = QFileDialog.getOpenFileName(self, "Select Second Point Cloud (PLY)", "",
                                                     "PLY Files (*.ply)", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not cloud2_file:
            QMessageBox.critical(self, "Error", "No second point cloud selected.")
            return

        # Select calibration file
        calibration_file, _ = QFileDialog.getOpenFileName(self, "Select Calibration File (JSON)", "",
                                                          "JSON Files (*.json)", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not calibration_file:
            QMessageBox.critical(self, "Error", "No calibration file selected.")
            return

        # Get voxel size and region radius parameters
        voxel_size, ok = QInputDialog.getDouble(self, "Voxel Size",
                                                "Enter voxel size (e.g., 0.01, range 0.001 - 0.1):", 0.01, 0.001,
                                                0.1, 3)
        if not ok:
            return

        self.update_status("Processing and merging point clouds using ICP...")
        # Save the resulting cloud
        merged_filename, _ = QFileDialog.getSaveFileName(self, "Save Merged Point Cloud to File", "",
                                                         "PLY Files (*.ply)",
                                                         options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not merged_filename:
            QMessageBox.critical(self, "Error", "No file selected for saving.")
            return
        if not merged_filename.lower().endswith(".ply"):
            merged_filename += ".ply"
        try:
            cloud1 = load_point_cloud(cloud1_file)
            cloud2 = load_point_cloud(cloud2_file)
            cloud1 = cloud1.voxel_down_sample(voxel_size)
            cloud2 = cloud2.voxel_down_sample(voxel_size)
            algorithm(cloud1, cloud2, calibration_file, merged_filename, voxel_size)

            QMessageBox.information(self, "Success", f"Merged point cloud saved at: {merged_filename}")
            resulting_cloud = load_point_cloud(merged_filename)
            # Update visualization
            self.vis.clear_geometries()
            self.vis.add_geometry(resulting_cloud)
            self.pcd = resulting_cloud
            self.update_status("Ready")

        except Exception as e:
            self.update_status("An error occurred while merging point clouds.")
            QMessageBox.critical(self, "Error", f"An error occurred: {e}")

    # Changes the granularity of a point cloud based on user input.
    def start_change_granularity(self):

        ply_file, _ = QFileDialog.getOpenFileName(self, "Select .ply File to Change Granularity", "", "PLY Files (*.ply)",
                                                  options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not ply_file:
            QMessageBox.critical(self, "Error", "No .ply file selected.")
            return

        voxel_size, ok = QInputDialog.getDouble(self, "Voxel Size", "Enter voxel size (range 0.001 - 0.1):", 0.01, 0.001, 0.1, 3)

        if not ok:
            return

        output_file, _ = QFileDialog.getSaveFileName(self, "Save Point Cloud with Changed Granularity", "", "PLY Files (*.ply)",
                                                     options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not output_file.lower().endswith(".ply"):
            output_file += ".ply"
        if not output_file:
            return

        self.update_status("Changing point cloud granularity...")

        try:
            change_granularity_of_point_cloud(ply_file, voxel_size, output_file)
            self.update_status(f"Granularity changed and saved as {output_file}")
            QMessageBox.information(self, "Success", f"Granularity changed and saved as {output_file}")
        except Exception as e:
            self.update_status("An error occurred while changing granularity.")
            QMessageBox.critical(self, "Error", f"An error occurred while changing granularity: {e}")

    # Performs stereo calibration based on user-selected parameters and images.
    def start_stereo_calibration(self):

        json1, _ = QFileDialog.getOpenFileName(self, "Select JSON File for First Camera Parameters", "", "JSON Files (*.json)",
                                              options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        json2, _ = QFileDialog.getOpenFileName(self, "Select JSON File for Second Camera Parameters", "", "JSON Files (*.json)",
                                              options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not json1 or not json2:
            QMessageBox.critical(self, "Error", "No JSON files selected.")
            return

        intrinsics1 = load_intrinsics(json1)
        intrinsics2 = load_intrinsics(json2)

        images1_folder = QFileDialog.getExistingDirectory(self, "Select Image Folder for Camera 1",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        images2_folder = QFileDialog.getExistingDirectory(self, "Select Image Folder for Camera 2",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not images1_folder or not images2_folder:
            QMessageBox.critical(self, "Error", "No image folders selected.")
            return

        images1_pattern = os.path.join(images1_folder, "*.png")
        images2_pattern = os.path.join(images2_folder, "*.png")

        square_size, ok = QInputDialog.getDouble(self, "Square Size", "Enter the size of the pattern square in mm:", 1.0, 0.01, 100.0, 2)
        if not ok:
            return

        self.update_status("Performing stereo calibration...")

        try:
            R, T = stereo_calibrate(images1_pattern, images2_pattern, (9, 6), intrinsics1["color"], intrinsics2["color"], square_size)
            if R is not None and T is not None:
                translation_distance = calculate_translation_distance(T)
                rotation_angle = calculate_rotation_angle(R)

                QMessageBox.information(self, "Calibration Completed", f"Distance: {translation_distance:.2f} mm\nRotation Angle: {rotation_angle:.2f} degrees")

                stereo_calibration_data = {
                    "R": R.tolist(),
                    "T": T.tolist(),
                    "Translation Distance": translation_distance,
                    "Rotation Angle": rotation_angle
                }

                output_stereo_json_filename, _ = QFileDialog.getSaveFileName(self, "Save Stereo Calibration Data", "", "JSON Files (*.json)",
                                                                            options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
                if output_stereo_json_filename:
                    with open(output_stereo_json_filename, 'w') as json_file:
                        json.dump(stereo_calibration_data, json_file, indent=4)
                    self.update_status(f"Calibration data saved to file: {output_stereo_json_filename}")
                    QMessageBox.information(self, "Success", f"Calibration data saved to file: {output_stereo_json_filename}")
            else:
                self.update_status("Stereo calibration failed.")
                QMessageBox.critical(self, "Error", "Stereo calibration failed.")
        except Exception as e:
            self.update_status("An error occurred during calibration.")
            QMessageBox.critical(self, "Error", f"An error occurred: {e}")

    # Performs stereo calibration for ZED cameras based on user-selected parameters and images.
    def start_zed_stereo_calibration(self):

        left_conf, _ = QFileDialog.getOpenFileName(self, "Select .conf File for Left Camera", "",
                                                   "CONF Files (*.conf)", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        right_conf, _ = QFileDialog.getOpenFileName(self, "Select .conf File for Right Camera", "",
                                                    "CONF Files (*.conf)", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not left_conf or not right_conf:
            QMessageBox.critical(self, "Error", "No .conf files selected for both cameras.")
            return

        images1_folder = QFileDialog.getExistingDirectory(self, "Select Image Folder for Left Camera", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        images2_folder = QFileDialog.getExistingDirectory(self, "Select Image Folder for Right Camera", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not images1_folder or not images2_folder:
            QMessageBox.critical(self, "Error", "No image folders selected.")
            return

        square_size, ok = QInputDialog.getDouble(self, "Square Size", "Enter the size of the square in mm:", 1.0, 0.01,
                                                 100.0, 2)
        if not ok:
            return

        self.update_status("Performing stereo calibration for ZED cameras...")

        intrinsics1 = load_intrinsics_from_conf(left_conf, "LEFT_CAM_HD")
        intrinsics2 = load_intrinsics_from_conf(right_conf, "RIGHT_CAM_HD")

        images1_pattern = os.path.join(images1_folder, "*.png")
        images2_pattern = os.path.join(images2_folder, "*.png")

        try:
            R, T = stereo_calibrate(images1_pattern, images2_pattern, (9, 6), intrinsics1, intrinsics2, square_size)
            if R is not None and T is not None:
                translation_distance = calculate_translation_distance(T)
                rotation_angle = calculate_rotation_angle(R)

                QMessageBox.information(self, "ZED Calibration Completed",
                                        f"Distance: {translation_distance:.2f} mm\nRotation Angle: {rotation_angle:.2f} degrees")

                stereo_calibration_data = {
                    "R": R.tolist(),
                    "T": T.tolist(),
                    "Translation Distance": translation_distance,
                    "Rotation Angle": rotation_angle
                }

                output_stereo_json_filename, _ = QFileDialog.getSaveFileName(self, "Save ZED Stereo Calibration Data",
                                                                             "", "JSON Files (*.json)")
                if output_stereo_json_filename:
                    with open(output_stereo_json_filename, 'w') as json_file:
                        json.dump(stereo_calibration_data, json_file, indent=4)
                    self.update_status(f"ZED calibration data saved to file: {output_stereo_json_filename}")
                    QMessageBox.information(self, "Success",
                                            f"ZED calibration data saved to file: {output_stereo_json_filename}")
            else:
                QMessageBox.critical(self, "Error", "ZED stereo calibration failed.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {e}")

    # Starts capturing from a single ZED camera based on user input.

    # Starts capturing from dual ZED cameras.
    def start_dual_zed_cameras(self):

        # Create the main folder for saved images
        base_name, ok = QInputDialog.getText(self, "Folder Name", "Enter the main folder name for images:")
        if not ok or not base_name:
            return

        main_folder = os.path.join(base_name, "Dual_ZED_Captures")
        os.makedirs(main_folder, exist_ok=True)

        # Start capturing in a separate thread
        threading.Thread(target=open_zed_camera, args=(base_name,)).start()

    # Calls the function to filter the point cloud and displays it in Open3D.
    def filter_and_show_point_cloud(self):


        ply_file, _ = QFileDialog.getOpenFileName(self, "Select .ply File to Crop", "",
                                                  "PLY Files (*.ply)",
                                                  options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not ply_file:
            QMessageBox.critical(self, "Error", "No .ply file selected.")
            return
        output_file, _ = QFileDialog.getSaveFileName(self, "Save Cropped Point Cloud", "",
                                                     "PLY Files (*.ply)",
                                                     options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not output_file.lower().endswith(".ply"):
            output_file += ".ply"
        if not output_file:
            return

        # Set maximum distance (you can also allow the user to input this)
        max_distance, ok = QInputDialog.getDouble(self,
                                                  "Maximum Distance",
                                                  "Enter maximum distance (range 0.5 - 3.0):",
                                                  1.7,  # Default value
                                                  0.5,  # Minimum value
                                                  3.0,  # Maximum value
                                                  2)
        if not ok:
            return

        # Call the filtering function
        filter_point_cloud(ply_file, output_file, max_distance)

        # Load the filtered point cloud
        self.pcd = o3d.io.read_point_cloud(output_file)

        # Update visualization
        self.vis.clear_geometries()
        self.vis.add_geometry(self.pcd)

        # Update status bar
        self.status_bar.showMessage(f"Point cloud filtered and saved to {output_file}")

    # Removes outliers from the point cloud and updates the visualization
    def remove_outlisers_from_cloude(self):

        ply_file, _ = QFileDialog.getOpenFileName(self, "Select .ply File to Clean", "",
                                                  "PLY Files (*.ply)",
                                                  options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not ply_file:
            QMessageBox.critical(self, "Error", "No .ply file selected.")
            return
        output_file, _ = QFileDialog.getSaveFileName(self, "Save Cleaned Point Cloud", "",
                                                     "PLY Files (*.ply)",
                                                     options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not output_file.lower().endswith(".ply"):
            output_file += ".ply"
        if not output_file:
            return

        cloud = load_point_cloud(ply_file)
        # Call the function to remove outliers
        cloud = remove_isolated_points(cloud, nb_neighbors=80, std_ratio=0.9)
        save_point_cloud_to_ply(cloud, output_file)
        # Load the cleaned point cloud
        self.pcd = o3d.io.read_point_cloud(output_file)

        # Update visualization
        self.vis.clear_geometries()
        self.vis.add_geometry(self.pcd)

        # Update status bar
        self.status_bar.showMessage(f"Point cloud cleaned and saved to {output_file}")

    # Captures and processes intrinsic parameters for RealSense cameras
    def get_cloud_and_intristics_realsense(self):

        output_ply_filename, _ = QFileDialog.getSaveFileName(self, "Save Point Cloud (PLY)", "",
                                                             "PLY Files (*.ply)",
                                                             options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not output_ply_filename.lower().endswith(".ply"):
            output_ply_filename += ".ply"
        if not output_ply_filename:
            return

        json1, _ = QFileDialog.getSaveFileName(self, "Save JSON File for RealSense Camera", "",
                                               "JSON Files (*.json)",
                                               options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not json1.lower().endswith(".json"):
            json1 += ".json"
        if not json1:
            return
        capture_and_process_intrinsics(output_ply_filename, json1)


    # Captures and processes intrinsic parameters for ZED cameras
    def get_cloud_and_intriscs_zed(self):

        output_ply_filename, _ = QFileDialog.getSaveFileName(self, "Save Point Cloud (PLY)", "",
                                                             "PLY Files (*.ply)",
                                                             options=QFileDialog.Option(
                                                                 QFileDialog.DontUseNativeDialog))
        if not output_ply_filename.lower().endswith(".ply"):
            output_ply_filename += ".ply"
        if not output_ply_filename:
            return

        conf1, _ = QFileDialog.getSaveFileName(self, "Save CONF File for ZED Camera", "",
                                               "CONF Files (*.conf)",
                                               options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not conf1.lower().endswith(".json"):
            conf1 += ".json"
        if not conf1:
            return
        start_zed_cameras(output_ply_filename, conf1)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PointCloudApp()
    window.show()
    sys.exit(app.exec())
