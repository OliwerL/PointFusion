import threading
import time
import os
import pyzed.sl as sl
import cv2


# Function to capture images from the left lens of a ZED camera
def capture_zed_camera(serial_number, main_folder, disable_depth=False):
    # Initialize ZED camera with a specific serial number
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Set to 1080p
    init_params.camera_fps = 15  # Set FPS to 15
    init_params.set_from_serial_number(serial_number)  # Set the camera by serial number

    # Optional depth mode adjustment for testing
    if disable_depth:
        init_params.depth_mode = sl.DEPTH_MODE.NONE  # Disable depth mode for testing

    # Open the camera
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print(f"Unable to open ZED camera with serial number {serial_number}.")
        return

    runtime_parameters = sl.RuntimeParameters()
    n = 1
    close_value = True

    # Create a folder for each camera inside the main folder
    camera_folder = os.path.join(main_folder, f"zed_{serial_number}")
    if not os.path.exists(camera_folder):
        os.makedirs(camera_folder)
        print(f"Folder '{camera_folder}' created.")

    # Capture loop
    while close_value:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            left_image = sl.Mat()
            zed.retrieve_image(left_image, sl.VIEW.LEFT)  # Retrieve left image
            left_image_np = left_image.get_data()  # Convert to numpy array

            # Display the left image
            cv2.imshow(f"ZED Camera {serial_number} - Left Lens", left_image_np)

            # Keypress event handling
            k = cv2.waitKey(5)
            if k == 27:  # ESC key
                close_value = False
            elif k == ord('s'):  # 's' key to save the image
                image_path = os.path.join(camera_folder, f'img{n}.png')
                cv2.imwrite(image_path, left_image_np)
                print(f"Image {n} from ZED {serial_number} saved to {image_path}")
                n += 1

    # Release resources
    zed.close()
    cv2.destroyAllWindows()


# Main program
def open_zed_camera(first_camera, second_camera):
    # List of ZED camera serial numbers
    zed_serial_numbers = [first_camera, second_camera]

    # Main folder for saving images
    main_folder = "dual_zed_images"
    if not os.path.exists(main_folder):
        os.makedirs(main_folder)
        print(f"Folder '{main_folder}' created.")

    # Start the first camera
    threads = []
    thread1 = threading.Thread(target=capture_zed_camera, args=(zed_serial_numbers[0], main_folder))
    thread1.start()
    threads.append(thread1)

    # Increased delay before starting the second camera
    time.sleep(10)  # Increase delay as needed

    # Start the second camera with depth mode disabled for testing
    thread2 = threading.Thread(target=capture_zed_camera, args=(zed_serial_numbers[1], main_folder, True))
    thread2.start()
    threads.append(thread2)

    # Wait for both threads to complete
    for thread in threads:
        thread.join()


if __name__ == "__main__":
    open_zed_camera(20007889,23041913)
