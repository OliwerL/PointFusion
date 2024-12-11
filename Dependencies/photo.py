import cv2
import os


# Captures photos from a single camera and saves them to the specified folder
def capture_photos(camera_index, output_folder):
    close_value = True
    n = 1  # Photo number
    vidcap = cv2.VideoCapture(camera_index)  # Using dynamic camera index

    print("Press 's' - Save photo, 'ESC' - Exit")

    while close_value:
        success, img = vidcap.read()
        if success:
            cv2.imshow('Preview', img)

        k = cv2.waitKey(5)  # Wait time in milliseconds
        if k == 27:  # ESC key
            close_value = False
        elif k == ord('s'):  # 's' key
            # Save photo in folder
            image_path = os.path.join(output_folder, f"img_{n}.png")
            cv2.imwrite(image_path, img)
            print(f"Photo {n} saved as {image_path}")
            n += 1

    # Release camera resources
    vidcap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Example usage
    camera_index = 0  # Default camera index
    output_folder = "captured_images"
    capture_photos(camera_index, output_folder)
