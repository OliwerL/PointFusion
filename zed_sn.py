import threading
import time
import os
import pyzed.sl as sl
import cv2


# Automatically detects connected ZED cameras and retrieves their serial numbers
def get_zed_serial_numbers():
    available_devices = sl.Camera.get_device_list()

    # Collect serial numbers of available ZED cameras
    serial_numbers = [device.serial_number for device in available_devices]

    return serial_numbers


# Captures images from a single ZED camera and saves them to the specified folder
def capture_zed_camera(serial_number, main_folder, disable_depth=False):
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Set resolution to 720p
    init_params.camera_fps = 15  # Set FPS to 15
    init_params.set_from_serial_number(serial_number)  # Set camera by serial number

    # Optionally disable depth mode for testing
    if disable_depth:
        init_params.depth_mode = sl.DEPTH_MODE.NONE

    # Open the camera
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED camera with serial number {serial_number}.")
        return

    print(f"ZED camera {serial_number} opened successfully.")

    # Initialize runtime parameters
    runtime_parameters = sl.RuntimeParameters()
    n = 1
    close_value = True

    # Create a folder for the camera within the main folder
    camera_folder = os.path.join(main_folder, f"zed_{serial_number}")
    if not os.path.exists(camera_folder):
        os.makedirs(camera_folder)
        print(f"Folder '{camera_folder}' created.")

    # Capture loop
    while close_value:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left camera image
            left_image = sl.Mat()
            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            left_image_np = left_image.get_data()

            # Display the image in a window
            cv2.imshow(f"ZED Camera {serial_number} - Left Lens", left_image_np)

            # Handle key events
            k = cv2.waitKey(5)
            if k == 27:
                # ESC key pressed, exit the loop
                close_value = False
            elif k == ord('s'):
                # 's' key pressed, save the current frame as an image
                image_path = os.path.join(camera_folder, f'img{n}.png')
                cv2.imwrite(image_path, left_image_np)
                print(f"Image {n} from ZED camera {serial_number} saved to {image_path}")
                n += 1

    # Release camera resources and close all OpenCV windows
    zed.close()
    cv2.destroyAllWindows()


# Opens and captures images from all detected ZED cameras
def open_zed_camera(main_folder):
    zed_serial_numbers = get_zed_serial_numbers()

    if len(zed_serial_numbers) < 2:
        print("Less than two ZED cameras found. Ensure that two ZED cameras are connected.")
        return

    # Define the main folder for saving images
    if not os.path.exists(main_folder):
        os.makedirs(main_folder)
        print(f"Folder '{main_folder}' created.")

    # List to hold threads for each camera
    threads = []
    # Initialize and start threads for each ZED camera
    for serial_number in zed_serial_numbers:
        thread = threading.Thread(target=capture_zed_camera, args=(serial_number, main_folder))
        threads.append(thread)
        thread.start()

    # Wait for all threads to finish
    for thread in threads:
        thread.join()

if __name__ == "__main__":
    # Define the main folder for saving images
    main_folder = "zed_captured_images"
    open_zed_camera(main_folder)
