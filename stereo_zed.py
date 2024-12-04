import cv2
import numpy as np
import glob
import json
import configparser


# Loads camera intrinsic parameters from a .conf file
def load_intrinsics_from_conf(conf_filename, cam_section):
    config = configparser.ConfigParser()
    config.read(conf_filename)

    # Retrieve camera intrinsic parameters
    intrinsics = {
        'fx': float(config[cam_section]['fx']),
        'fy': float(config[cam_section]['fy']),
        'cx': float(config[cam_section]['cx']),
        'cy': float(config[cam_section]['cy']),
        'coeffs': [
            float(config[cam_section]['k1']),
            float(config[cam_section]['k2']),
            float(config[cam_section]['p1']),
            float(config[cam_section]['p2']),
            float(config[cam_section]['k3']),
        ]
    }
    return intrinsics


# Displays images with detected chessboard corners side by side, scaled for better visibility
def show_images_with_corners(image1, corners1, ret1, image2, corners2, ret2):
    scale_factor = 0.5  # Scale to 50% for easier viewing
    image1_resized = cv2.resize(image1, None, fx=scale_factor, fy=scale_factor)
    image2_resized = cv2.resize(image2, None, fx=scale_factor, fy=scale_factor)

    if ret1:
        cv2.drawChessboardCorners(image1_resized, (9, 6), corners1 * scale_factor, ret1)
    if ret2:
        cv2.drawChessboardCorners(image2_resized, (9, 6), corners2 * scale_factor, ret2)

    # Combine images horizontally
    combined_image = np.hstack((image1_resized, image2_resized))
    # Display the combined image
    cv2.imshow('Corners', combined_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Performs stereo calibration using pairs of checkerboard images
def stereo_calibrate(images1_pattern, images2_pattern, chessboard_size, intrinsics1, intrinsics2, square_size):
    # Define termination criteria for corner sub-pix
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points based on the checkerboard size and square size
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Initialize lists to store object points and image points from both cameras
    objpoints = []
    imgpoints1 = []
    imgpoints2 = []

    # Retrieve all image file paths matching the provided patterns
    images1 = glob.glob(images1_pattern)
    images2 = glob.glob(images2_pattern)

    # Check if the number of images matches
    if len(images1) != len(images2):
        print("Number of images in both patterns should be the same")
        return None, None

    # Iterate through image pairs for calibration
    for fname1, fname2 in zip(images1, images2):
        print(f"Processing pair: {fname1}, {fname2}")
        # Read images from both cameras
        img1 = cv2.imread(fname1)
        img2 = cv2.imread(fname2)
        if img1 is None or img2 is None:
            print(f"Error reading image pair: {fname1}, {fname2}")
            continue

        # Convert images to grayscale
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        # Find chessboard corners in both images
        ret1, corners1 = cv2.findChessboardCorners(gray1, chessboard_size,
                                                   cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        ret2, corners2 = cv2.findChessboardCorners(gray2, chessboard_size,
                                                   cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        # Display images with detected corners for verification
        show_images_with_corners(img1, corners1, ret1, img2, corners2, ret2)

        if ret1 and ret2:
            # Append object points and refined image points if corners are detected
            objpoints.append(objp)
            corners1 = cv2.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
            corners2 = cv2.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)
            imgpoints1.append(corners1)
            imgpoints2.append(corners2)
        else:
            print(f"Chessboard not found in pair: {fname1}, {fname2}")
            print(f"ret1: {ret1}, ret2: {ret2}")

    # Check if any valid pairs were found
    if len(objpoints) == 0:
        print("No valid image pairs found for calibration")
        return None, None

    # Construct camera matrices from intrinsic parameters
    mtx1 = np.array([[intrinsics1['fx'], 0, intrinsics1['cx']],
                     [0, intrinsics1['fy'], intrinsics1['cy']],
                     [0, 0, 1]])
    dist1 = np.array(intrinsics1['coeffs'])

    mtx2 = np.array([[intrinsics2['fx'], 0, intrinsics2['cx']],
                     [0, intrinsics2['fy'], intrinsics2['cy']],
                     [0, 0, 1]])
    dist2 = np.array(intrinsics2['coeffs'])

    # Perform stereo calibration
    ret, mtx1, dist1, mtx2, dist2, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints1, imgpoints2,
        mtx1, dist1, mtx2, dist2, gray1.shape[::-1],
        criteria=criteria)

    return R, T


# Saves stereo calibration results to a JSON file
def save_stereo_calibration(R, T, output_json_filename):
    stereo_calibration_data = {
        "R": R.tolist(),
        "T": T.tolist()
    }
    with open(output_json_filename, 'w') as json_file:
        json.dump(stereo_calibration_data, json_file, indent=4)


# Calculates the translation distance from the translation vector
def calculate_translation_distance(T):
    distance = np.linalg.norm(T)
    return distance


# Calculates the rotation angle from the rotation matrix
def calculate_rotation_angle(R):

    theta = np.arccos((np.trace(R) - 1) / 2)
    angle_degrees = np.degrees(theta)
    return angle_degrees


if __name__ == "__main__":
    # Sections for HD resolution
    left_cam_section = 'LEFT_CAM_HD'
    right_cam_section = 'LEFT_CAM_HD'
    right_conf_filename = "zed_sn_par/SN23509586.conf"  # Path to .conf file for the left camera
    left_conf_filename = "zed_sn_par/SN20749196.conf"  # Path to .conf file for the right camera

    # Load intrinsic parameters from separate .conf files for left and right cameras
    intrinsics1 = load_intrinsics_from_conf(left_conf_filename, left_cam_section)
    intrinsics2 = load_intrinsics_from_conf(right_conf_filename, right_cam_section)

    # Display loaded intrinsic parameters
    print("Intrinsics for left camera:", intrinsics1)
    print("Intrinsics for right camera:", intrinsics2)

    # Define output file for stereo calibration
    output_stereo_json_filename = "stereo_calibration.json"
    # Define image patterns for both cameras
    camera1 = "dual_zed_images/zed_20007889/*.png"
    camera2 = "dual_zed_images/zed_23041913/*.png"
    # Define the size of a checkerboard square in millimeters
    square_size = 65  # Size of one checkerboard square in mm

    # Perform stereo calibration
    R, T = stereo_calibrate(camera1, camera2, (9, 6), intrinsics1, intrinsics2, square_size)

    if R is not None and T is not None:
        # Save calibration results
        save_stereo_calibration(R, T, output_stereo_json_filename)
        print("Stereo calibration successful")
        print("R:", R)
        print("T:", T)

        # Calculate translation distance and rotation angle
        translation_distance = calculate_translation_distance(T)
        rotation_angle = calculate_rotation_angle(R)

        # Display calibration metrics
        print("Translation distance between cameras (T):", translation_distance)
        print("Rotation angle between cameras (R) in degrees:", rotation_angle)

        # Save additional calibration metrics to JSON file
        stereo_calibration_data = {
            "R": R.tolist(),
            "T": T.tolist(),
            "Translation Distance": translation_distance,
            "Rotation Angle": rotation_angle
        }

        with open(output_stereo_json_filename, 'w') as json_file:
            json.dump(stereo_calibration_data, json_file, indent=4)
    else:
        print("Stereo calibration failed")
