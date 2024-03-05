import open3d as o3d
import numpy as np

def load_point_cloud(fp):
    ld = np.load(fp)
    return ld['arr_0']

def crop_point_cloud(pc, si):
    if not si:
        print("No bounding points selected. Exiting.")
        return None
    
    sp = pc.select_by_index(si)
    mb = sp.get_min_bound() - 0.1 
    ma = sp.get_max_bound() + 0.1 
    bb = o3d.geometry.AxisAlignedBoundingBox(mb, ma)

    cpc = pc.crop(bb)
    return cpc

def find_rotation_matrix(nv1, nv2, nv3):
    xa = np.array([1, 0, 0])
    nv1[2] = 0
    a = np.arccos(np.dot(nv1, xa) / (np.linalg.norm(nv1) * np.linalg.norm(xa)))
    print(a)
    
    ya = np.array([0, 1, 0])
    nv2[2] = 0
    b = np.arccos(np.dot(nv2, ya) / (np.linalg.norm(nv2) * np.linalg.norm(ya)))
    print(b)
    
    za = np.array([0, 0, 1])
    nv3[1] = 0
    c = np.arccos(np.dot(nv3, za) / (np.linalg.norm(nv3) * np.linalg.norm(za)))
    print(b)
    
    rm = np.array([[np.cos(a)*np.cos(b), np.cos(a)*np.sin(b)*np.sin(c)-np.sin(a)*np.cos(c), np.cos(a)*np.sin(b)*np.cos(c)+np.sin(a)*np.sin(c)],
                   [np.sin(a)*np.cos(b), np.sin(a)*np.sin(b)*np.sin(c)+np.cos(a)*np.cos(c), np.sin(a)*np.sin(b)*np.cos(c)-np.cos(a)*np.sin(c)],
                   [-np.sin(b), np.cos(b)*np.sin(c), np.cos(b)*np.cos(c)]])
    print("Rotation Matrix:")
    print(rm)
    return rm

def visualize_point_cloud(pc):
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(pc)

    v = o3d.visualization.VisualizerWithEditing()
    v.create_window()
    v.add_geometry(p)
    v.run() 
    v.destroy_window()
    
    si = v.get_picked_points()  
    return si, p

def segment_plane(pc):
    pm, i = pc.segment_plane(distance_threshold=0.01,
                              ransac_n=4,
                              num_iterations=1000)
    [a, b, c, d] = pm
    return a, b, c, i

def trans_vec():
    """
    manual measurement:

    lidar - vehicle
    x = 85cm
    y = 0cm
    z = 168cm

    """
    translation_vector = np.array([0.85, 0, 1.68])
    return translation_vector

if __name__ == "__main__":
    file_path = '/Users/ananaymathur/Desktop/lidar1.npz'
    point_cloud = load_point_cloud(file_path)
    nv1 = np.array(3)
    nv2 = np.array(3)
    for i in range(3):
        opt = input("opt: ")
        selected_indices, pcd = visualize_point_cloud(point_cloud)
        if pcd:
            cropped_pcd = crop_point_cloud(pcd, selected_indices)
            
            if cropped_pcd:
                a, b, c, inliers = segment_plane(cropped_pcd)
                
                if inliers:
                    wall_plane = cropped_pcd.select_by_index(inliers)
                    
                    if opt == '1':
                        nv1 = np.array([a, b, c])
                        print("Normal vector of wall:", nv1)
                    if opt == '2':
                        nv2 = np.array([a, b, c])
                        print("Normal vector of wall:", nv2)
                    if opt == '3':
                        nv3 = np.array([a, b, c])
                        print("Normal vector of wall:", nv3)
                    
                    
                    
    rotation_matrix = find_rotation_matrix(nv1, nv2, nv3)
    translation_vector = trans_vec()
    
    transform_matrix = np.eye(4)  
    transform_matrix[:3, :3] = rotation_matrix  
    transform_matrix[:3, 3] = translation_vector

    print("Transformation Matrix:")
    print(transform_matrix)
