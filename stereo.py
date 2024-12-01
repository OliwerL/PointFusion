import cv2
import numpy as np
import glob
import json
import os

def load_intrinsics(json_filename):
    with open(json_filename, 'r') as f:
        data = json.load(f)
    return data


def show_images_with_corners(image1, corners1, ret1, image2, corners2, ret2):
    if ret1:
        cv2.drawChessboardCorners(image1, (9, 6), corners1, ret1)
    if ret2:
        cv2.drawChessboardCorners(image2, (9, 6), corners2, ret2)
    combined_image = np.hstack((image1, image2))
    cv2.imshow('Corners', combined_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def stereo_calibrate(images1_pattern, images2_pattern, chessboard_size, intrinsics1, intrinsics2, square_size):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size  # Skalowanie współrzędnych o rozmiar kwadratu

    objpoints = []
    imgpoints1 = []
    imgpoints2 = []

    images1 = glob.glob(images1_pattern)
    images2 = glob.glob(images2_pattern)

    if len(images1) != len(images2):
        print("Number of images in both patterns should be the same")
        return None, None

    for fname1, fname2 in zip(images1, images2):
        print(f"Processing pair: {fname1}, {fname2}")
        img1 = cv2.imread(fname1)
        img2 = cv2.imread(fname2)
        if img1 is None or img2 is None:
            print(f"Error reading image pair: {fname1}, {fname2}")
            continue

        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        ret1, corners1 = cv2.findChessboardCorners(gray1, chessboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        ret2, corners2 = cv2.findChessboardCorners(gray2, chessboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        # Dodaj diagnostykę obrazów
        show_images_with_corners(img1, corners1, ret1, img2, corners2, ret2)

        if ret1 and ret2:
            objpoints.append(objp)
            corners1 = cv2.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
            corners2 = cv2.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)
            imgpoints1.append(corners1)
            imgpoints2.append(corners2)
        else:
            print(f"Chessboard not found in pair: {fname1}, {fname2}")
            print(f"ret1: {ret1}, ret2: {ret2}")

    if len(objpoints) == 0:
        print("No valid image pairs found for calibration")
        return None, None

    mtx1 = np.array([[intrinsics1['fx'], 0, intrinsics1['cx']],
                     [0, intrinsics1['fy'], intrinsics1['cy']],
                     [0, 0, 1]])
    dist1 = np.array(intrinsics1['coeffs'])

    mtx2 = np.array([[intrinsics2['fx'], 0, intrinsics2['cx']],
                     [0, intrinsics2['fy'], intrinsics2['cy']],
                     [0, 0, 1]])
    dist2 = np.array(intrinsics2['coeffs'])

    ret, mtx1, dist1, mtx2, dist2, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints1, imgpoints2,
        mtx1, dist1, mtx2, dist2, gray1.shape[::-1],
        criteria=criteria)

    return R, T

def save_stereo_calibration(R, T, output_json_filename):
    stereo_calibration_data = {
        "R": R.tolist(),
        "T": T.tolist()
    }
    with open(output_json_filename, 'w') as json_file:
        json.dump(stereo_calibration_data, json_file, indent=4)

def calculate_translation_distance(T):
    # Obliczanie odległości (długości wektora translacji)
    distance = np.linalg.norm(T)
    return distance


def calculate_rotation_angle(R):
    # Obliczanie kąta obrotu
    theta = np.arccos((np.trace(R) - 1) / 2)
    return np.degrees(theta)  # Kąt w stopniach

if __name__ == "__main__":
    name1 = "dalsza_"
    name2 = "glowna_dalsza_"
    intrinsics1 = load_intrinsics(name1 + "JSON.json")
    intrinsics2 = load_intrinsics(name2 + "JSON.json")

    # Sprawdzenie wczytanych parametrów
    print("Intrinsics for camera 1 (color):", intrinsics1["color"])
    print("Intrinsics for camera 2 (color):", intrinsics2["color"])

    output_stereo_json_filename = name1 + name2 +"STEREO.json"
    camera1 = name1 + 'ZDJ/*.png'
    camera2 = name2 + 'ZDJ/*.png'
    square_size = 65  # rozmiar jednego kwadratu w mm

    # Przykład użycia
    R, T = stereo_calibrate(camera1, camera2, (9, 6), intrinsics1["color"], intrinsics2["color"], square_size)

    if R is not None and T is not None:
        save_stereo_calibration(R, T, output_stereo_json_filename)
        print("Stereo calibration successful")
        print("R", R)
        print("T", T)
    else:
        print("Stereo calibration failed")

    # Obliczanie wartości
    if R is not None and T is not None:
        translation_distance = calculate_translation_distance(T)
        rotation_angle = calculate_rotation_angle(R)

        # Wyświetlanie wyników
        print("Odległość między kamerami (T):", translation_distance)
        print("Kąt obrotu między kamerami (R) w stopniach:", rotation_angle)

        # Dodanie tych wartości do pliku JSON
        stereo_calibration_data = {
            "R": R.tolist(),
            "T": T.tolist(),
            "Translation Distance": translation_distance,
            "Rotation Angle": rotation_angle
        }

        with open(output_stereo_json_filename, 'w') as json_file:
            json.dump(stereo_calibration_data, json_file, indent=4)
    else:
        print("Nie można obliczyć odległości i kąta obrotu bez wyników kalibracji stereo")
