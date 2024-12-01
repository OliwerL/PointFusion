import open3d as o3d


def zmien_granulacje_chmury_punktow(nazwa_pliku, rozmiar_voxela, nowa_nazwa_pliku):
    # Wczytaj chmurę punktów z pliku .ply
    chmura_punktow = o3d.io.read_point_cloud(nazwa_pliku)
    print(f"Liczba punktów przed zmniejszeniem granulacji: {len(chmura_punktow.points)}")

    # Zastosuj Voxel Downsampling do zmniejszenia liczby punktów
    chmura_punktow_zmieniona = chmura_punktow.voxel_down_sample(voxel_size=rozmiar_voxela)
    print(f"Liczba punktów po zmniejszeniu granulacji: {len(chmura_punktow_zmieniona.points)}")

    # Zapisz nową chmurę punktów do pliku .ply
    o3d.io.write_point_cloud(nowa_nazwa_pliku, chmura_punktow_zmieniona)
    print(f"Nowa chmura punktów zapisana jako {nowa_nazwa_pliku}")

if __name__ == "__main__":
    # Użycie funkcji:
    nazwa_pliku = "srodek.ply"
    rozmiar_voxela = 0.01  # Ustaw rozmiar voxela w zależności od potrzeb
    nowa_nazwa_pliku = "srodek_orignal.ply"

    zmien_granulacje_chmury_punktow(nazwa_pliku, rozmiar_voxela, nowa_nazwa_pliku)
