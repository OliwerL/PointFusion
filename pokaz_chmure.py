import open3d as o3d
import numpy as np

def visualize_point_cloud(ply_file):
    # Wczytaj chmurę punktów z pliku PLY
    pcd = o3d.io.read_point_cloud(ply_file)

    # Sprawdź, czy chmura punktów została poprawnie wczytana
    if pcd.is_empty():
        print("Chmura punktów jest pusta.")
        return


    # Wyświetl chmurę punktów
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    # Ścieżka do pliku PLY
    # ply_file = 'prawyPLY.ply'
    ply_file = 'te_dwie_co_chciales.ply'
    ply_file1 = 'te_dwie_co_chciales_test.ply'
    # ply_file2 = '44_realsens.ply'
    chmura_punktow = o3d.io.read_point_cloud(ply_file)
    visualize_point_cloud(ply_file)
    chmura_punktow1 = o3d.io.read_point_cloud(ply_file1)
    visualize_point_cloud(ply_file1)
    # chmura_punktow2 = o3d.io.read_point_cloud(ply_file2)
    # visualize_point_cloud(ply_file2)
    print(f"Liczba punktów w oryginalnej chmurze: {len(chmura_punktow.points)}")
    print(f"Liczba punktów w oryginalnej chmurze: {len(chmura_punktow1.points)}")
    # print(f"Liczba punktów w oryginalnej chmurze: {len(chmura_punktow2.points)}")

