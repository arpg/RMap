import re
import os
import argparse
import open3d as o3d
import numpy as np


def parse_txt(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()
    

    points = []
    #pattern = r"Transform\s*{\s*translation\s*([\d\.\-]+)\s+([\d\.\-]+)\s+([\d\.\-]+)\s*"
    pattern = r"-?\d+\.\d+"

    for l in lines:
        num = re.findall(pattern, l)
        num = [float(i) for i in num]
        points.append([num[0], num[1], num[2]])
        
    return points

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--maps_dir', type = str, help = 'Base folder contatning entire lidar and radar octomaps(.txt)')
    args = parser.parse_args()

    files = sorted(os.listdir(args.maps_dir))
    mapfiles = [f for f in files if f.endswith(".txt")]
    out_folder = os.path.join(os.path.dirname(maps_dir), "pcd")
    if not os.path.exists(out_folder):
        os.makedirs(out_folder)

    for filepath in mapfiles:
        
        points = parse_txt(os.path.join(maps_dir, filepath))


        points = np.array(points)
        print("Parsed 3D Points:", points.shape)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        output_file = os.path.join(out_folder, filepath.split(".")[0] + ".ply")
        print(filepath, output_file)

        kdtree = o3d.geometry.KDTreeFlann(pcd)
        count = 0
        for pt in pcd.points:
            [k, idx, _] = kdtree.search_knn_vector_3d(pt, 2)
            target_pt = np.asarray(pcd.points)[idx[1], :]
            diff = np.abs(np.array(pt) - target_pt)
            # if(diff[0]==0.15 or diff[0]==0.0 or diff[1]==0.15 or diff[1]==0.0 or diff[2]==0.15 or diff[2]==0.0):
            #     continue
            # else:
            #     #print(diff)
            count+=1
        print(count, len(pcd.points))

        o3d.io.write_point_cloud(output_file, pcd, True, True)

