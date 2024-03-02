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

def find_rotation_matrix(nv):
    xa = np.array([1, 0, 0])
    a = np.arccos(np.dot(nv, xa) / (np.linalg.norm(nv) * np.linalg.norm(xa)))
    rm = np.array([[np.cos(a), -np.sin(a), 0],
                   [np.sin(a), np.cos(a), 0],
                   [0, 0, 1]])
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
    print(f"Plane function: {a}x + {b}y + {c}z + {d} = 0")
    return a, b, c, i

if __name__ == "__main__":
    file_path = ''
    point_cloud = load_point_cloud(file_path)
    
    selected_indices, pcd = visualize_point_cloud(point_cloud)
    
    if pcd:
        cropped_pcd = crop_point_cloud(pcd, selected_indices)
        
        if cropped_pcd:
            a, b, c, inliers = segment_plane(cropped_pcd)
            
            if inliers:
                wall_plane = cropped_pcd.select_by_index(inliers)
                normal_vector = np.array([a, b, c])
                print("Normal vector of wall:", normal_vector)
                
                rotation_matrix = find_rotation_matrix(normal_vector)
