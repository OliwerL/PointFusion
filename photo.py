import cv2
import os

def capture_photos(camera_index, output_folder):
    close_value = True
    n = 1  # Numer zdjęcia
    vidcap = cv2.VideoCapture(camera_index)  # Używamy dynamicznego indeksu kamery

    print("Klawisz 's' - Zapisz zdjęcie, 'ESC' - Wyjście")

    while close_value:
        success, img = vidcap.read()
        if success:
            cv2.imshow('Preview', img)

        k = cv2.waitKey(5)  # Czas oczekiwania w milisekundach
        if k == 27:  # ESC key
            close_value = False
        elif k == ord('s'):  # 's' key
            # Zapis zdjęcia w folderze
            image_path = os.path.join(output_folder, f"img_{n}.png")
            cv2.imwrite(image_path, img)
            print(f"Zdjęcie nr {n} zapisane jako {image_path}")
            n += 1

    # Zwolnij zasoby kamery
    vidcap.release()
    cv2.destroyAllWindows()
