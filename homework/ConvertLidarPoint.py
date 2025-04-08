import numpy as np
import os

def load_lidar_from_npz(npz_file):
    """
    从 npz 文件中加载点云数据，假设数据存储在 'points' 键下，
    如果没有 'points' 键，则取第一个键对应的数据。
    """
    data = np.load(npz_file)
    if 'points' in data:
        return data['points']
    else:
        key = list(data.keys())[0]
        return data[key]

def save_points_to_ply(points, ply_filename):
    """
    将点云数据保存为 ASCII 格式的 PLY 文件。
    """
    N = points.shape[0]
    with open(ply_filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("element vertex {}\n".format(N))
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in points:
            f.write("{:.6f} {:.6f} {:.6f}\n".format(p[0], p[1], p[2]))

if __name__ == '__main__':
    # 例如，转换 lidar_top1.npz 到 lidar_top5.npz
    for i in range(90, 98):
        npz_filename = f'../parking_data/lidar_top{i}.npz'
        ply_filename = f'../parking_data/lidar_top{i}.ply'
        lidar_points = load_lidar_from_npz(npz_filename)
        save_points_to_ply(lidar_points, ply_filename)
        print(f"转换完成：{npz_filename} -> {ply_filename}")