import sys
import os
import threading
import json
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget, QPushButton, QVBoxLayout,
    QHBoxLayout, QLabel, QFileDialog, QMessageBox, QInputDialog, QStatusBar
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QWindow
from PySide6.QtGui import QFont
import win32gui
from cloude_filtering import filter_point_cloud


# Importowanie funkcji z plików
from bag_to_ply import extract_point_cloud_from_bag, convert_to_open3d_point_cloud, save_point_cloud_to_ply
from photo import capture_photos
from multiphoto import multi_camera
from parameters import get_intrinsics_from_bag
from merge_clouds import load_point_cloud, apply_transformation, merge_point_clouds, load_calibration_data
from granularity import zmien_granulacje_chmury_punktow
from stereo import load_intrinsics, stereo_calibrate, calculate_translation_distance, calculate_rotation_angle
import open3d as o3d  # Make sure to import open3d if used
from stereo_zed import stereo_calibrate, calculate_translation_distance, calculate_rotation_angle, load_intrinsics_from_conf
from zed_sn import capture_zed_camera, open_zed_camera
from merge_point_clouds_icp import algorithm



class PointCloudApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Point Cloud GUI")
        # Zmienna przechowująca wybór kamery (0 = ZED, 1 = RealSense)
        self.camera_type = self.choose_camera_type()

        # Jeśli użytkownik anulował wybór, zamknij aplikację
        if self.camera_type is None:
            sys.exit()

        self.setWindowTitle("Point Cloud GUI")
        self.setGeometry(100, 100, 1600, 800)

        # Konfiguracja głównego layoutu
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # Tworzenie zakładek i przycisków po lewej stronie
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
                    QTabBar::tab {
                        font-size: 9pt;
                    }
                    QTabBar {
                        qproperty-expanding: 1;  /* Wymusza rozciąganie zakładek na całą szerokość */
                    }
                """)
        self.create_tabs()
        main_layout.addWidget(self.tabs, stretch=1)

        # Konfiguracja Open3D po prawej stronie
        self.pcd = o3d.io.read_point_cloud("filtered_output.ply")  # Domyślnie załadowana chmura punktów
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.add_geometry(self.pcd)

        # Pobranie uchwytu okna Open3D do integracji z PySide6
        hwnd = win32gui.FindWindowEx(0, 0, None, "Open3D")
        self.o3d_window = QWindow.fromWinId(hwnd)
        self.o3d_container = self.createWindowContainer(self.o3d_window, main_widget)
        main_layout.addWidget(self.o3d_container, stretch=2)

        # Timer do odświeżania wizualizacji Open3D
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_vis)
        self.timer.start(16)  # Odświeżanie co ~16 ms (około 60 FPS)

        # Pasek stanu
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Gotowy")

    def choose_camera_type(self):
        items = ["ZED", "RealSense"]
        camera_choice, ok = QInputDialog.getItem(
            self,
            "Wybór kamery",
            "Wybierz typ kamery, z której chcesz korzystać:",
            items,
            0,
            False,
        )
        if ok:
            return 0 if camera_choice == "ZED" else 1
        else:
            return None

    def create_tabs(self):
        """Dodaje wszystkie zakładki do QTabWidget."""
        # Inicjalizacja poszczególnych zakładek
        self.processing_tab = QWidget()
        self.visualization_tab = QWidget()
        self.camera_tab = QWidget()
        self.segmentation_tab = QWidget()
        self.calibration_tab = QWidget()

        # Ustawienia zakładek
        self.create_processing_tab()
        self.create_visualization_tab()
        self.create_camera_tab()
        # self.create_segmentation_tab()
        self.create_calibration_tab()

        # Dodanie zakładek do QTabWidget
        self.tabs.addTab(self.processing_tab, "Przetwarzanie")
        self.tabs.addTab(self.visualization_tab, "Wizualizacja")
        self.tabs.addTab(self.camera_tab, "Kamery")
        # self.tabs.addTab(self.segmentation_tab, "Segmentacja")
        self.tabs.addTab(self.calibration_tab, "Kalibracja")




    def create_processing_tab(self):
        layout = QVBoxLayout()
        button_font = QFont('Helvetica', 11)

        if self.camera_type == 1:  # RealSense
            self.bag_to_ply_button = QPushButton("Konwertuj BAG na PLY")
            self.bag_to_ply_button.setFont(button_font)
            self.bag_to_ply_button.clicked.connect(self.process_bag_to_ply)
            layout.addWidget(self.bag_to_ply_button)

            self.intrinsics_button = QPushButton("Zapisz parametry intrynsyczne")
            self.intrinsics_button.setFont(button_font)
            self.intrinsics_button.clicked.connect(self.save_intrinsics)
            layout.addWidget(self.intrinsics_button)

            self.merge_clouds_button = QPushButton("Połącz chmury punktów")
            self.merge_clouds_button.setFont(button_font)
            self.merge_clouds_button.clicked.connect(self.start_merge_point_clouds)
            layout.addWidget(self.merge_clouds_button)

        self.change_granularity_button = QPushButton("Zmień granulację chmury punktów")
        self.change_granularity_button.setFont(button_font)
        self.change_granularity_button.clicked.connect(self.start_change_granularity)
        layout.addWidget(self.change_granularity_button)

        if self.camera_type == 0:
            self.merge_clouds_icp_button = QPushButton("Połącz chmury punktów z ICP")
            self.merge_clouds_icp_button.setFont(button_font)
            self.merge_clouds_icp_button.clicked.connect(self.start_merge_point_clouds_with_icp)
            layout.addWidget(self.merge_clouds_icp_button)

        self.processing_tab.setLayout(layout)

    def create_visualization_tab(self):
        layout = QVBoxLayout()
        button_font = QFont('Helvetica', 11)

        self.show_cloud_button = QPushButton("Wyświetl chmurę")
        self.show_cloud_button.setFont(button_font)
        self.show_cloud_button.clicked.connect(self.show_point_cloud)
        layout.addWidget(self.show_cloud_button)

        # Przycisk do filtrowania chmury
        self.filter_cloud_button = QPushButton("Obetnij chmurę")
        self.filter_cloud_button.setFont(button_font)
        self.filter_cloud_button.clicked.connect(self.filter_and_show_point_cloud)
        layout.addWidget(self.filter_cloud_button)

        self.visualization_tab.setLayout(layout)

    def create_camera_tab(self):
        layout = QVBoxLayout()
        button_font = QFont('Helvetica', 11)

        if self.camera_type == 1:  # RealSense
            self.camera_button = QPushButton("Uruchom pojedynczą kamerę")
            self.camera_button.setFont(button_font)
            self.camera_button.clicked.connect(self.start_camera)
            layout.addWidget(self.camera_button)

            self.multi_camera_button = QPushButton("Uruchom dwie kamery")
            self.multi_camera_button.setFont(button_font)
            self.multi_camera_button.clicked.connect(self.start_multi_camera_capture)
            layout.addWidget(self.multi_camera_button)

        if self.camera_type == 0:
            self.single_zed_button = QPushButton("Uruchom pojedynczą kamerę")
            self.single_zed_button.setFont(button_font)
            self.single_zed_button.clicked.connect(self.start_single_zed_camera)
            layout.addWidget(self.single_zed_button)

            self.dual_zed_button = QPushButton("Uruchom dwie kamery")
            self.dual_zed_button.setFont(button_font)
            self.dual_zed_button.clicked.connect(self.start_dual_zed_cameras)
            layout.addWidget(self.dual_zed_button)

        self.camera_tab.setLayout(layout)


    def create_calibration_tab(self):
        layout = QVBoxLayout()
        button_font = QFont('Helvetica', 11)

        if self.camera_type == 1:  # RealSense
            self.stereo_calibration_button = QPushButton("Kalibracja stereo")
            self.stereo_calibration_button.setFont(button_font)
            self.stereo_calibration_button.clicked.connect(self.start_stereo_calibration)
            layout.addWidget(self.stereo_calibration_button)
        if self.camera_type == 0:  # RealSense
            self.zed_calibration_button = QPushButton("Kalibracja stereo")
            self.zed_calibration_button.setFont(button_font)
            self.zed_calibration_button.clicked.connect(self.start_zed_stereo_calibration)
            layout.addWidget(self.zed_calibration_button)

        self.calibration_tab.setLayout(layout)

    def update_vis(self):
        """Odświeża wizualizację Open3D."""
        self.vis.poll_events()
        self.vis.update_renderer()

    def update_status(self, message):
        self.status_bar.showMessage(message)
        QApplication.processEvents()

    def process_bag_to_ply(self):
        # Wybór pliku .bag
        bag_path, _ = QFileDialog.getOpenFileName(self, "Wybierz plik .bag", "", "BAG Files (*.bag)",
                                                  options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if bag_path:
            # Użytkownik wybiera nazwę pliku .ply do zapisu
            ply_filename, _ = QFileDialog.getSaveFileName(self, "Zapisz plik .ply", "", "PLY Files (*.ply)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
            if ply_filename:
                if not ply_filename.lower().endswith(".ply"):
                    ply_filename += ".ply"
                # Aktualizacja paska stanu
                self.update_status("Przetwarzanie pliku .bag na .ply...")
                # Wywołanie funkcji z bag_to_ply.py
                color_frame, depth_frame = extract_point_cloud_from_bag(bag_path)
                self.point_cloud = convert_to_open3d_point_cloud(color_frame, depth_frame)
                save_point_cloud_to_ply(self.point_cloud, ply_filename)
                self.update_status(f"Chmura punktów zapisana do pliku: {ply_filename}")

                # Wyświetlenie informacji o sukcesie
                QMessageBox.information(self, "Sukces", f"Chmura punktów zapisana do pliku: {ply_filename}")

    def show_point_cloud(self):
        """Pozwala wybrać plik .ply i wyświetla go w oknie Open3D."""
        ply_path, _ = QFileDialog.getOpenFileName(self, "Wybierz plik .ply", "", "PLY Files (*.ply)",
                                                  options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if ply_path:
            self.update_status("Ładowanie wybranej chmury punktów...")
            new_pcd = o3d.io.read_point_cloud(ply_path)
            self.vis.clear_geometries()  # Usuwa poprzednią chmurę punktów
            self.vis.add_geometry(new_pcd)  # Dodaje nową chmurę punktów
            self.pcd = new_pcd  # Zaktualizowanie self.pcd
            self.update_status(f"Wyświetlono chmurę punktów: {ply_path}")
        else:
            QMessageBox.critical(self, "Błąd", "Nie wybrano pliku .ply")

    def start_camera(self):
        QMessageBox.information(self, "Informacja", "Klawisz 's' - Zapisz zdjęcie\nKlawisz 'ESC' - Wyjście z podglądu kamery")

        camera_index, ok = QInputDialog.getInt(self, "Indeks kamery", "Podaj indeks kamery (np. 0, 1, 2):", min=0)
        if not ok:
            return

        base_name, ok = QInputDialog.getText(self, "Nazwa folderu", "Podaj nazwę folderu dla zdjęć:")
        if not ok or not base_name:
            return

        output_folder = f"{base_name}_ZDJ"
        os.makedirs(output_folder, exist_ok=True)

        self.update_status("Uruchamianie kamery...")

        threading.Thread(target=capture_photos, args=(camera_index, output_folder)).start()

    def save_intrinsics(self):
        bag_path, _ = QFileDialog.getOpenFileName(self, "Wybierz plik .bag", "", "BAG Files (*.bag)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if bag_path:
            json_filename, _ = QFileDialog.getSaveFileName(self, "Zapisz plik .json", "", "JSON Files (*.json)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
            if json_filename:
                if not json_filename.lower().endswith(".json"):
                    json_filename += ".json"
                self.update_status("Zapisywanie parametrów intrynsycznych...")
                get_intrinsics_from_bag(bag_path, json_filename)
                self.update_status(f"Parametry intrynsyczne zapisane do pliku: {json_filename}")
                QMessageBox.information(self, "Sukces", f"Parametry intrynsyczne zapisane do pliku: {json_filename}")

    def start_multi_camera_capture(self):
        indices_input, ok = QInputDialog.getText(self, "Indeksy kamer", "Podaj indeksy kamer, oddzielone przecinkami (np. 0,1,2):")
        if not ok or not indices_input:
            return

        try:
            camera_indices = [int(idx.strip()) for idx in indices_input.split(",")]
        except ValueError:
            QMessageBox.critical(self, "Błąd", "Nieprawidłowy format indeksów kamer.")
            return

        base_name, ok = QInputDialog.getText(self, "Nazwa folderu", "Podaj nazwę folderu dla zdjęć z wielokamerowego przechwytywania:")
        if not ok or not base_name:
            return

        output_folder = f"{base_name}_MultiCamera_ZDJ"
        os.makedirs(output_folder, exist_ok=True)

        self.update_status("Uruchamianie wielokamerowego przechwytywania...")

        threading.Thread(target=multi_camera, args=(camera_indices, output_folder)).start()

    def start_merge_point_clouds(self):
        num_clouds, ok = QInputDialog.getInt(self, "Liczba chmur", "Podaj liczbę chmur do połączenia:", 2, 2, 100)

        if not ok:
            return

        ply_filenames = []
        for i in range(num_clouds):
            ply_filename, _ = QFileDialog.getOpenFileName(self, f"Wybierz plik .ply dla chmury {i + 1}", "",
                                                          "PLY Files (*.ply)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
            if not ply_filename:
                QMessageBox.critical(self, "Błąd", "Nie wybrano pliku PLY dla jednej z chmur.")
                return
            ply_filenames.append(ply_filename)

        calibration_files = []
        for i in range(1, num_clouds):
            json_filename, _ = QFileDialog.getOpenFileName(
                self,
                f"Wybierz plik kalibracyjny .json dla transformacji chmury {i + 1} do chmury 1",
                "",
                "JSON Files (*.json)",
                options=QFileDialog.Option(QFileDialog.DontUseNativeDialog)
            )
            if not json_filename:
                QMessageBox.critical(self, "Błąd", "Nie wybrano pliku JSON dla jednej z transformacji.")
                return
            calibration_files.append(json_filename)

        clouds = []
        # Zapytanie o granularność dla każdej chmury
        voxel_size, ok = QInputDialog.getDouble(
            self,
            f"Rozmiar voxela dla chmury {i + 1}",
            "Podaj rozmiar voxela dla tej chmury (np. 0.01, zakres 0.001 - 0.1):",
            0.01, 0.001, 0.1, 3
        )
        if not ok:
            return

        for i, ply_filename in enumerate(ply_filenames):
            point_cloud = load_point_cloud(ply_filename)

            try:
                point_cloud = point_cloud.voxel_down_sample(voxel_size)  # Zmiana granularności dla tej chmury
                self.update_status(f"Granulacja chmury {i + 1} ustawiona na {voxel_size}")
            except Exception as e:
                QMessageBox.critical(self, "Błąd", f"Nie udało się zmienić granulacji dla chmury {i + 1}: {e}")
                return

            if i == 0:
                clouds.append(point_cloud)
            else:
                R, T = load_calibration_data(calibration_files[i - 1])
                transformed_cloud = apply_transformation(point_cloud, R, T)
                clouds.append(transformed_cloud)

        # Po zmianie granularności połącz chmury
        merged_cloud = merge_point_clouds(clouds)


        merged_filename, _ = QFileDialog.getSaveFileName(self, "Zapisz połączoną chmurę punktów", "",
                                                         "PLY Files (*.ply)",
                                                         options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not merged_filename.lower().endswith(".ply"):
            merged_filename += ".ply"
        if not merged_filename:
            return

        o3d.io.write_point_cloud(merged_filename, merged_cloud)
        self.update_status(f"Chmury punktów połączone i zapisane do pliku: {merged_filename}")
        QMessageBox.information(self, "Sukces", f"Chmury punktów połączone i zapisane do pliku: {merged_filename}")

        # Zaktualizuj wizualizację
        self.update_status("Aktualizowanie wizualizacji...")
        self.vis.clear_geometries()  # Usunięcie starej chmury punktów z wizualizatora
        self.vis.add_geometry(merged_cloud)  # Dodanie nowej, zmergowanej chmury
        self.pcd = merged_cloud  # Zaktualizowanie referencji do chmury punktów
        self.update_status("Gotowy")

    def start_merge_point_clouds_with_icp(self):
        # Pobierz pliki chmur punktów
        cloud1_file,_ = QFileDialog.getOpenFileName(self, "Wybierz pierwszą chmurę punktów (PLY)", "",
                                                     "PLY Files (*.ply)", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not cloud1_file:
            QMessageBox.critical(self, "Błąd", "Nie wybrano pierwszej chmury punktów.")
            return

        cloud2_file, _ = QFileDialog.getOpenFileName(self, "Wybierz drugą chmurę punktów (PLY)", "",
                                                     "PLY Files (*.ply)", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not cloud2_file:
            QMessageBox.critical(self, "Błąd", "Nie wybrano drugiej chmury punktów.")
            return

        # Pobierz plik kalibracyjny
        calibration_file, _ = QFileDialog.getOpenFileName(self, "Wybierz plik kalibracyjny (JSON)", "",
                                                          "JSON Files (*.json)", options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not calibration_file:
            QMessageBox.critical(self, "Błąd", "Nie wybrano pliku kalibracyjnego.")
            return

        # Pobierz parametry granularności i promienia regionu centralnego
        voxel_size, ok = QInputDialog.getDouble(self, "Rozmiar voxela",
                                                "Podaj rozmiar voxela (np. 0.01, zakres 0.001 - 0.1):", 0.01, 0.001,
                                                0.1, 3)
        if not ok:
            return

        self.update_status("Przetwarzanie i łączenie chmur punktów z użyciem ICP...")
        # Zapisanie wynikowej chmury
        merged_filename, _ = QFileDialog.getSaveFileName(self, "Zapisz połączoną chmurę punktów do pliku", "",
                                                         "PLY Files (*.ply)",
                                                         options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not merged_filename:
            QMessageBox.critical(self, "Błąd", "Nie wybrano pliku do zapisu.")
            return
        if not merged_filename.lower().endswith(".ply"):
            merged_filename += ".ply"
        try:
            cloud1 = load_point_cloud(cloud1_file)
            cloud2 = load_point_cloud(cloud2_file)
            cloud1 = cloud1.voxel_down_sample(voxel_size)
            cloud2 = cloud2.voxel_down_sample(voxel_size)
            algorithm(cloud1, cloud2, calibration_file, merged_filename,voxel_size)


            QMessageBox.information(self, "Sukces", f"Połączona chmura punktów została zapisana w: {merged_filename}")
            resulting_cloud = load_point_cloud(merged_filename)
            # Aktualizacja wizualizacji
            self.vis.clear_geometries()
            self.vis.add_geometry(resulting_cloud)
            self.pcd = resulting_cloud
            self.update_status("Gotowy")

        except Exception as e:
            self.update_status("Wystąpił błąd podczas łączenia chmur punktów.")
            QMessageBox.critical(self, "Błąd", f"Wystąpił błąd: {e}")

    def start_change_granularity(self):
        ply_file, _ = QFileDialog.getOpenFileName(self, "Wybierz plik .ply do zmiany granulacji", "", "PLY Files (*.ply)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not ply_file:
            QMessageBox.critical(self, "Błąd", "Nie wybrano pliku .ply.")
            return

        voxel_size, ok = QInputDialog.getDouble(self, "Rozmiar przestrzeni zajmowanej przez jeden voxel", "Podaj rozmiar voksela z zakresu 0.001 - 0.1 :", 0.01, 0.001, 0.1, 3)

        if not ok:
            return

        output_file, _ = QFileDialog.getSaveFileName(self, "Zapisz chmurę punktów po zmianie granulacji", "", "PLY Files (*.ply)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not output_file.lower().endswith(".ply"):
            output_file += ".ply"
        if not output_file:
            return

        self.update_status("Zmiana granulacji chmury punktów...")

        try:
            zmien_granulacje_chmury_punktow(ply_file, voxel_size, output_file)
            self.update_status(f"Granulacja zmieniona i zapisana jako {output_file}")
            QMessageBox.information(self, "Sukces", f"Granulacja zmieniona i zapisana jako {output_file}")
        except Exception as e:
            self.update_status("Wystąpił błąd podczas zmiany granulacji.")
            QMessageBox.critical(self, "Błąd", f"Wystąpił błąd podczas zmiany granulacji: {e}")


    def start_stereo_calibration(self):
        json1, _ = QFileDialog.getOpenFileName(self, "Wybierz plik JSON z parametrami pierwszej kamery", "", "JSON Files (*.json)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        json2, _ = QFileDialog.getOpenFileName(self, "Wybierz plik JSON z parametrami drugiej kamery", "", "JSON Files (*.json)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not json1 or not json2:
            QMessageBox.critical(self, "Błąd", "Nie wybrano plików JSON.")
            return

        intrinsics1 = load_intrinsics(json1)
        intrinsics2 = load_intrinsics(json2)

        images1_folder = QFileDialog.getExistingDirectory(self, "Wybierz folder z obrazami dla kamery 1",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        images2_folder = QFileDialog.getExistingDirectory(self, "Wybierz folder z obrazami dla kamery 2",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not images1_folder or not images2_folder:
            QMessageBox.critical(self, "Błąd", "Nie wybrano folderów z obrazami.")
            return

        images1_pattern = os.path.join(images1_folder, "*.png")
        images2_pattern = os.path.join(images2_folder, "*.png")

        square_size, ok = QInputDialog.getDouble(self, "Rozmiar kwadratu", "Podaj rozmiar kwadratu wzorca w mm:", 1.0, 0.01, 100.0, 2)
        if not ok:
            return

        self.update_status("Kalibracja stereo...")

        try:
            R, T = stereo_calibrate(images1_pattern, images2_pattern, (9, 6), intrinsics1["color"], intrinsics2["color"], square_size)
            if R is not None and T is not None:
                translation_distance = calculate_translation_distance(T)
                rotation_angle = calculate_rotation_angle(R)

                QMessageBox.information(self, "Kalibracja zakończona", f"Odległość: {translation_distance:.2f} mm\nKąt obrotu: {rotation_angle:.2f} stopni")

                stereo_calibration_data = {
                    "R": R.tolist(),
                    "T": T.tolist(),
                    "Translation Distance": translation_distance,
                    "Rotation Angle": rotation_angle
                }

                output_stereo_json_filename, _ = QFileDialog.getSaveFileName(self, "Zapisz dane kalibracji stereo", "", "JSON Files (*.json)",
                                                          options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
                if output_stereo_json_filename:
                    with open(output_stereo_json_filename, 'w') as json_file:
                        json.dump(stereo_calibration_data, json_file, indent=4)
                    self.update_status(f"Dane kalibracyjne zapisane do pliku: {output_stereo_json_filename}")
                    QMessageBox.information(self, "Sukces", f"Dane kalibracyjne zapisane do pliku: {output_stereo_json_filename}")
            else:
                self.update_status("Kalibracja stereo nie powiodła się.")
                QMessageBox.critical(self, "Błąd", "Kalibracja stereo nie powiodła się.")
        except Exception as e:
            self.update_status("Wystąpił błąd podczas kalibracji.")
            QMessageBox.critical(self, "Błąd", f"Wystąpił błąd: {e}")

    def start_zed_stereo_calibration(self):
        left_conf, _ = QFileDialog.getOpenFileName(self, "Wybierz plik .conf dla lewej kamery", "",
                                                   "CONF Files (*.conf)")
        right_conf, _ = QFileDialog.getOpenFileName(self, "Wybierz plik .conf dla prawej kamery", "",
                                                    "CONF Files (*.conf)")
        if not left_conf or not right_conf:
            QMessageBox.critical(self, "Błąd", "Nie wybrano plików .conf dla obu kamer.")
            return

        images1_folder = QFileDialog.getExistingDirectory(self, "Wybierz folder z obrazami dla lewej kamery")
        images2_folder = QFileDialog.getExistingDirectory(self, "Wybierz folder z obrazami dla prawej kamery")
        if not images1_folder or not images2_folder:
            QMessageBox.critical(self, "Błąd", "Nie wybrano folderów z obrazami.")
            return

        square_size, ok = QInputDialog.getDouble(self, "Rozmiar kwadratu", "Podaj rozmiar kwadratu w mm:", 1.0, 0.01,
                                                 100.0, 2)
        if not ok:
            return

        self.update_status("Kalibracja stereo dla kamer ZED...")

        intrinsics1 = load_intrinsics_from_conf(left_conf, "LEFT_CAM_HD")
        intrinsics2 = load_intrinsics_from_conf(right_conf, "RIGHT_CAM_HD")

        images1_pattern = os.path.join(images1_folder, "*.png")
        images2_pattern = os.path.join(images2_folder, "*.png")

        try:
            R, T = stereo_calibrate(images1_pattern, images2_pattern, (9, 6), intrinsics1, intrinsics2, square_size)
            if R is not None and T is not None:
                translation_distance = calculate_translation_distance(T)
                rotation_angle = calculate_rotation_angle(R)

                QMessageBox.information(self, "Kalibracja ZED zakończona",
                                        f"Odległość: {translation_distance:.2f} mm\nKąt obrotu: {rotation_angle:.2f} stopni")

                stereo_calibration_data = {
                    "R": R.tolist(),
                    "T": T.tolist(),
                    "Translation Distance": translation_distance,
                    "Rotation Angle": rotation_angle
                }

                output_stereo_json_filename, _ = QFileDialog.getSaveFileName(self, "Zapisz dane kalibracji ZED stereo",
                                                                             "", "JSON Files (*.json)")
                if output_stereo_json_filename:
                    with open(output_stereo_json_filename, 'w') as json_file:
                        json.dump(stereo_calibration_data, json_file, indent=4)
                    self.update_status(f"Dane kalibracyjne ZED zapisane do pliku: {output_stereo_json_filename}")
                    QMessageBox.information(self, "Sukces",
                                            f"Dane kalibracyjne ZED zapisane do pliku: {output_stereo_json_filename}")
            else:
                QMessageBox.critical(self, "Błąd", "Kalibracja stereo ZED nie powiodła się.")
        except Exception as e:
            QMessageBox.critical(self, "Błąd", f"Wystąpił błąd: {e}")

    def start_single_zed_camera(self):
        # Pobranie numeru seryjnego kamery
        serial, ok = QInputDialog.getInt(self, "Numer seryjny kamery", "Podaj numer seryjny kamery ZED:")
        if not ok:
            return

        # Wyłączenie trybu głębi (opcjonalne)
        disable_depth, ok = QInputDialog.getItem(
            self,
            "Tryb głębi",
            "Czy wyłączyć tryb głębi?",
            ["Nie", "Tak"],
            0,
            False,
        )
        if not ok:
            return

        disable_depth_flag = disable_depth == "Tak"

        # Tworzenie folderu dla zapisanych obrazów
        base_name, ok = QInputDialog.getText(self, "Nazwa folderu", "Podaj nazwę głównego folderu dla obrazów:")
        if not ok or not base_name:
            return

        main_folder = os.path.join(base_name, "ZED_Captures")
        os.makedirs(main_folder, exist_ok=True)

        # Uruchomienie kamery w wątku
        threading.Thread(target=capture_zed_camera, args=(serial, main_folder, disable_depth_flag)).start()

    def start_dual_zed_cameras(self):
        # Pobranie numerów seryjnych kamer
        serial1, ok1 = QInputDialog.getInt(self, "Numer seryjny kamery 1", "Podaj numer seryjny pierwszej kamery:")
        if not ok1:
            return

        serial2, ok2 = QInputDialog.getInt(self, "Numer seryjny kamery 2", "Podaj numer seryjny drugiej kamery:")
        if not ok2:
            return

        # Tworzenie folderu głównego dla zapisanych obrazów
        base_name, ok = QInputDialog.getText(self, "Nazwa folderu", "Podaj nazwę głównego folderu dla obrazów:")
        if not ok or not base_name:
            return

        main_folder = os.path.join(base_name, "Dual_ZED_Captures")
        os.makedirs(main_folder, exist_ok=True)

        # Uruchomienie przechwytywania w wątku
        threading.Thread(target=open_zed_camera, args=(serial1, serial2)).start()


    def filter_and_show_point_cloud(self):
        """
        Funkcja wywołująca filtrowanie chmury punktów i wyświetlanie jej w Open3D.
        """
        ply_file, _ = QFileDialog.getOpenFileName(self, "Wybierz plik .ply do zmiany granulacji", "",
                                                  "PLY Files (*.ply)",
                                                  options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not ply_file:
            QMessageBox.critical(self, "Błąd", "Nie wybrano pliku .ply.")
            return
        output_file, _ = QFileDialog.getSaveFileName(self, "Zapisz chmurę punktów po zmianie granulacji", "",
                                                     "PLY Files (*.ply)",
                                                     options=QFileDialog.Option(QFileDialog.DontUseNativeDialog))
        if not output_file.lower().endswith(".ply"):
            output_file += ".ply"
        if not output_file:
            return

        # Ustal maksymalną odległość (można też pozwolić użytkownikowi na jej wprowadzenie)
        max_distance, ok = QInputDialog.getDouble(self,
                                                  "Maksymalna odległość",
                                                  "Podaj maksymalną odległość (zakres 0.5 - 3.0):",
                                                  1.7,  # Domyślna wartość
                                                  0.5,  # Minimalna wartość
                                                  3.0,  # Maksymalna wartość
                                                  2)
        if not ok:
            return

        # Wywołaj funkcję filtrującą
        filter_point_cloud(ply_file, output_file, max_distance)

        # Wczytaj przefiltrowaną chmurę punktów
        self.pcd = o3d.io.read_point_cloud(output_file)

        # Aktualizuj wizualizację
        self.vis.clear_geometries()
        self.vis.add_geometry(self.pcd)

        # Zaktualizuj pasek stanu
        self.status_bar.showMessage(f"Chmura punktów została przefiltrowana i zapisana do {output_file}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PointCloudApp()
    window.show()
    sys.exit(app.exec())
