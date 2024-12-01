import cv2
import threading
import os

# Funkcja obsługująca każdą kamerę
def capture_camera(camera_index, folder_name):
    vidcap = cv2.VideoCapture(camera_index)  # Otwieranie kamery na podstawie indeksu
    n = 1
    close_value = True

    while close_value:
        succes, img = vidcap.read()
        if succes:
            cv2.imshow(f'Preview Camera {camera_index}', img)

        k = cv2.waitKey(5)
        if k == 27:  # ESC key
            close_value = False
        elif k == ord('s'):  # 's' key
            image_path = os.path.join(folder_name, f'img_camera{camera_index}_{n}.png')
            cv2.imwrite(image_path, img)
            print(f"Image {n} from Camera {camera_index} saved to {image_path}")
            n += 1

    vidcap.release()
    cv2.destroyAllWindows()

# Funkcja do obsługi wielu kamer
def multi_camera(camera_indices, folder_name):
    # Tworzenie folderu, jeśli nie istnieje
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
        print(f"Folder '{folder_name}' created.")

    # Lista wątków
    threads = []

    # Inicjalizacja i uruchamianie wątków dla każdej kamery
    for idx in camera_indices:
        thread = threading.Thread(target=capture_camera, args=(idx, folder_name))
        threads.append(thread)
        thread.start()

    # Oczekiwanie na zakończenie wszystkich wątków
    for thread in threads:
        thread.join()

if __name__ == "__main__":
    # Przykład z domyślnymi wartościami
    default_indices = [0, 1]  # Indeksy kamer
    default_folder = "default_images_folder"
    multi_camera(default_indices, default_folder)
