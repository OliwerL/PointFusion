import threading
import time
import os
import pyzed.sl as sl
import cv2


# Funkcja do automatycznego wykrywania kamer ZED
def get_zed_serial_numbers():
    # Inicjalizacja ZED SDK i lista urządzeń
    available_devices = sl.Camera.get_device_list()

    # Zbieranie numerów seryjnych dostępnych kamer ZED
    serial_numbers = [device.serial_number for device in available_devices]

    return serial_numbers


# Funkcja do przechwytywania obrazów z kamery ZED
def capture_zed_camera(serial_number, main_folder, disable_depth=False):
    # Inicjalizacja kamery ZED z numerem seryjnym
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Ustawienie rozdzielczości 720p
    init_params.camera_fps = 15  # Ustawienie FPS na 15
    init_params.set_from_serial_number(serial_number)  # Ustawienie kamery po numerze seryjnym

    # Opcjonalne wyłączenie trybu głębokości (dla testów)
    if disable_depth:
        init_params.depth_mode = sl.DEPTH_MODE.NONE  # Wyłączenie trybu głębokości

    # Otwarcie kamery
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print(f"Nie udało się otworzyć kamery ZED o numerze seryjnym {serial_number}.")
        return

    runtime_parameters = sl.RuntimeParameters()
    n = 1
    close_value = True

    # Tworzenie folderu dla każdej kamery wewnątrz folderu głównego
    camera_folder = os.path.join(main_folder, f"zed_{serial_number}")
    if not os.path.exists(camera_folder):
        os.makedirs(camera_folder)
        print(f"Utworzono folder '{camera_folder}'.")

    # Pętla przechwytywania
    while close_value:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            left_image = sl.Mat()
            zed.retrieve_image(left_image, sl.VIEW.LEFT)  # Pobranie obrazu z lewego obiektywu
            left_image_np = left_image.get_data()  # Konwersja do tablicy numpy

            # Wyświetlanie obrazu
            cv2.imshow(f"ZED Camera {serial_number} - Left Lens", left_image_np)

            # Obsługa zdarzenia klawisza
            k = cv2.waitKey(5)
            if k == 27:  # Jeśli naciśnięty klawisz ESC
                close_value = False
            elif k == ord('s'):  # Jeśli naciśnięty klawisz 's', zapisz obraz
                image_path = os.path.join(camera_folder, f'img{n}.png')
                cv2.imwrite(image_path, left_image_np)
                print(f"Obraz {n} z kamery ZED {serial_number} zapisany do {image_path}")
                n += 1

    # Zwolnienie zasobów
    zed.close()
    cv2.destroyAllWindows()


# Funkcja główna do otwierania kamer ZED
def open_zed_camera(main_folder):
    # Pobranie dostępnych numerów seryjnych kamer ZED
    zed_serial_numbers = get_zed_serial_numbers()

    if len(zed_serial_numbers) < 2:
        print("Nie znaleziono dwóch kamer ZED. Upewnij się, że są podłączone.")
        return

    # Główny folder do zapisywania obrazów
    main_folder = main_folder
    if not os.path.exists(main_folder):
        os.makedirs(main_folder)
        print(f"Folder '{main_folder}' został utworzony.")

    # Uruchomienie pierwszej kamery
    threads = []
    thread1 = threading.Thread(target=capture_zed_camera, args=(zed_serial_numbers[0], main_folder))
    thread1.start()
    threads.append(thread1)

    # Zwiększenie opóźnienia przed uruchomieniem drugiej kamery
    time.sleep(10)  # Zwiększ opóźnienie w razie potrzeby

    # Uruchomienie drugiej kamery z wyłączonym trybem głębokości (dla testów)
    thread2 = threading.Thread(target=capture_zed_camera, args=(zed_serial_numbers[1], main_folder))
    thread2.start()
    threads.append(thread2)

    # Czekanie na zakończenie obu wątków
    for thread in threads:
        thread.join()


if __name__ == "__main__":
    open_zed_camera()
