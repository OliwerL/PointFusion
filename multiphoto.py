import cv2
import threading
import os


# Captures photos from a single camera and saves them to the specified folder
def capture_camera(camera_index, folder_name):

    vidcap = cv2.VideoCapture(camera_index)  # Open camera based on index
    n = 1
    close_value = True

    print(f"Press 's' - Save photo from Camera {camera_index}, 'ESC' - Exit")

    while close_value:
        success, img = vidcap.read()
        if success:
            cv2.imshow(f'Preview Camera {camera_index}', img)

        k = cv2.waitKey(5)  # Wait time in milliseconds
        if k == 27:  # ESC key
            close_value = False
        elif k == ord('s'):  # 's' key
            # Save photo in folder
            image_path = os.path.join(folder_name, f'img_camera{camera_index}_{n}.png')
            cv2.imwrite(image_path, img)
            print(f"Image {n} from Camera {camera_index} saved to {image_path}")
            n += 1

    # Release camera resources
    vidcap.release()
    cv2.destroyWindow(f'Preview Camera {camera_index}')

# Captures photos from multiple cameras simultaneously and saves them to the specified folder.
def multi_camera(camera_indices, folder_name):
    # Create folder if it doesn't exist

    folder1_name = f"{folder_name}_cam{camera_indices[0]}"
    folder2_name = f"{folder_name}_cam{camera_indices[1]}"

    if not os.path.exists(folder1_name):
        os.makedirs(folder1_name)
        print(f"Folder '{folder1_name}' created.")

    if not os.path.exists(folder2_name):
        os.makedirs(folder2_name)
        print(f"Folder '{folder2_name}' created.")

    # List to hold threads
    threads = []

    # Initialize and start threads for each camera
    for idx in camera_indices:
        thread = threading.Thread(target=capture_camera, args=(idx, folder_name))
        threads.append(thread)
        thread.start()

    # Wait for all threads to finish
    for thread in threads:
        thread.join()

if __name__ == "__main__":
    # Example usage with default values
    default_indices = [0, 1]  # Camera indices
    default_folder = "default_images_folder"
    multi_camera(default_indices, default_folder)
