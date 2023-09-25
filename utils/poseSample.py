import os
import sys
import time
from datetime import datetime
import open3d as o3d
import math
from scipy.spatial import cKDTree, KDTree
from sklearn.cluster import KMeans
import numpy as np
import matplotlib.pyplot as plt
import copy 
import argparse



def calculate_resolution(points):
    # Compute the Euclidean distance between neighboring points
    kdtree = cKDTree(points)
    _, indices = kdtree.query(points, k=2)  # Find the nearest 2 points for each point
    distances = np.linalg.norm(points[indices[:, 1]] - points[indices[:, 0]], axis=1)

    # Calculate the average distance as the resolution
    resolution = np.mean(distances)
    
    return resolution

def get_anchor_points(pcd, seed_point, num_points):
    points =np.array(pcd.points)
    points_2d = points[:,:2]
    kdtree = KDTree(points_2d)
    [k, idx] = kdtree.query(seed_point, num_points)
    patch_points = np.asarray(points)[idx, :]
    patch_pcd = o3d.geometry.PointCloud()
    patch_pcd.points = o3d.utility.Vector3dVector(patch_points)
    pcd_fps = patch_pcd.farthest_point_down_sample(4)

    anchor_points = [seed_point]
    for pt in pcd_fps.points:
        anchor_points.append(pt[:2])
    return anchor_points

def geodesic_growth(lidar_pcd, radar_pcd, seed_point, num_points, ratio):
    lidar_patches = []
    radar_patches = []
    
    anchor_pts = get_anchor_points(lidar_pcd, seed_point, num_points*4)
    for pt in anchor_pts:
        points = np.array(lidar_pcd.points)
        points_2d = points[:,:2]
        kdtree = KDTree(points_2d)
        [k, idx] = kdtree.query(pt, num_points)
        lidar_patch_points = np.asarray(points)[idx, :]
        lidar_patch_pcd = o3d.geometry.PointCloud()
        lidar_patch_pcd.points = o3d.utility.Vector3dVector(lidar_patch_points)
        if len(lidar_patches) >0:
            index, d = minDistance(lidar_patches, lidar_patch_pcd)
            if(d<250):
                continue
        lidar_patches.append(lidar_patch_pcd)

        points = np.array(radar_pcd.points)
        points_2d = points[:,:2]
        kdtree = KDTree(points_2d)
        [k, idx] = kdtree.query(pt, int(num_points*ratio))
        radar_patch_points = np.asarray(points)[idx, :]
        radar_patch_pcd = o3d.geometry.PointCloud()
        radar_patch_pcd.points = o3d.utility.Vector3dVector(radar_patch_points)
        radar_patches.append(radar_patch_pcd)
        

    return lidar_patches, radar_patches

def get_seed_points(filepath):
    filepath = filepath.replace("/pcd","/poses")
    filepath = filepath.replace("_lidar_filtered.ply",".txt")
    points = np.loadtxt(filepath, delimiter=',')
    points = points[:,:2]
    seed_points = [points[0]]
    for i in range(1, points.shape[0]):
        includeFlag = True
        for pt in seed_points:
            d = np.linalg.norm(pt - points[i])
            if(d<0.9):
                includeFlag = False
                break
        if(includeFlag):
            seed_points.append(points[i])
    seed_points = np.array(seed_points)
    print(seed_points.shape, points.shape)
    x = seed_points[:,0]
    y = seed_points[:,1]
    
    fig, ax = plt.subplots()
    ax.plot(x, y)
    fig.savefig(filepath.replace(".txt", "_odom_xy.png"))
    
    return seed_points



def minDistance(prev_patches, patch):
    distances = [np.sum(prev.compute_point_cloud_distance(patch)) for prev in prev_patches]
    distances = np.array(distances)
    idx = np.argmin(distances)
    return idx, distances[idx]
       
if __name__ == "__main__":
    
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd_dir', type = str, help = 'Base folder contatning entire lidar and radar pointclouds')
    parser.add_argument('--gt_dir', type = str, help = 'Groundtruth folder to store the generated pointclouds')
    parser.add_argument('--input_dir', type = str, help = 'Input folder to store the generated pointclouds')
    # parser.add_argument('--num_points', type=int, help='Number of points in each GT patch')
    # parser.add_argument('--ratio', type)
    num_points = 8192
    ratio = 1/4
    args = parser.parse_args()
    
    files = os.listdir(args.pcd_dir)
    lidar_files = sorted([os.path.join(args.pcd_dir, f) for f in files if f.endswith("lidar_filtered.ply")])
    radar_files = sorted([os.path.join(args.pcd_dir, f) for f in files if f.endswith("radar.ply")])

    
    if not os.path.exists(args.gt_dir):
        os.makedirs(args.gt_dir)
    

    if not os.path.exists(args.input_dir):
        os.makedirs(args.input_dir)
    

    for f in radar_files:
        radar_pcd = o3d.io.read_point_cloud(f)
        lidar_filename = f.replace("radar.ply", "lidar_filtered.ply")
        lidar_pcd = o3d.io.read_point_cloud(lidar_filename)
        prev_patches = None    
        print(f, lidar_filename)
        seed_points = get_seed_points(lidar_filename)
        radar_pcd.points = o3d.utility.Vector3dVector(radar_pcd,lidar_pcd)
        idx = 0
        for i in range(seed_points.shape[0]):
            lidar, radar = geodesic_growth(lidar_pcd, radar_pcd, seed_points[i], num_points, ratio)
            for j, patch in enumerate(lidar):
                gt_filename =  f"{os.path.basename(lidar_filename).split('.')[0]}_{idx:04d}.pcd"
                gt_filepath = os.path.join(args.gt_dir, gt_filename)
                gt_filepath = gt_filepath.replace("_lidar_filtered","")
                input_filename =  f"{os.path.basename(f).split('.')[0]}_{idx:04d}.pcd"
                input_filepath = os.path.join(args.input_dir, input_filename)
                input_filepath = input_filepath.replace("_radar","")
                if prev_patches is not None:
                    index, d = minDistance(prev_patches, patch)
                    if(d<5000):
                        continue
                    prev_patches[index] = patch
                o3d.io.write_point_cloud(gt_filepath, lidar[j], True, True)
                o3d.io.write_point_cloud(input_filepath, radar[j], True, True)
                idx += 1
            if prev_patches is None:
                prev_patches = copy.deepcopy(lidar)
        print(idx)
        

