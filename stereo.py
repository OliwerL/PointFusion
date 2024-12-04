import cv2
import numpy as np
import glob
import json


# Loads intrinsic parameters from a JSON file
def load_intrinsics(json_filename):

    with open(json_filename, 'r') as f:
        data = json.load(f)
    return data


# Displays images with detected chessboard corners side by side
def show_images_with_corners(image1, corners1, ret1, image2, corners2, ret2):
    if ret1:
        cv2.drawChessboardCorners(image1, (9, 6), corners1, ret1)
    if ret2:
        cv2.drawChessboardCorners(image2, (9, 6), corners2, ret2)
    # Combine images horizontally
    combined_image = np.hstack((image1, image2))
    # Display the combined image
    cv2.imshow('Corners', combined_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Performs stereo calibration using pairs of checkerboard images
def stereo_calibrate(images1_pattern, images2_pattern, chessboard_size, intrinsics1, intrinsics2, square_size):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Prepare object points based on the checkerboard size and square size
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size  # Scale object points by the size of a square

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
        ret1, corners1 = cv2.findChessboardCorners(gray1, chessboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        ret2, corners2 = cv2.findChessboardCorners(gray2, chessboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

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
    # Define camera configuration names
    name1 = "dalsza_"
    name2 = "glowna_dalsza_"
    # Load intrinsic parameters from JSON files
    intrinsics1 = load_intrinsics(name1 + "JSON.json")
    intrinsics2 = load_intrinsics(name2 + "JSON.json")

    # Display loaded intrinsic parameters
    print("Intrinsics for camera 1 (color):", intrinsics1["color"])
    print("Intrinsics for camera 2 (color):", intrinsics2["color"])

    # Define output file for stereo calibration
    output_stereo_json_filename = name1 + name2 + "STEREO.json"
    # Define image patterns for both cameras
    camera1 = name1 + 'ZDJ/*.png'
    camera2 = name2 + 'ZDJ/*.png'
    # Define the size of a checkerboard square in millimeters
    square_size = 65

    # Perform stereo calibration
    R, T = stereo_calibrate(camera1, camera2, (9, 6), intrinsics1["color"], intrinsics2["color"], square_size)

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
