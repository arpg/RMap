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

scene_names = [
                "arpg_lab_run0_", "arpg_lab_run1_", "arpg_lab_run2_", "arpg_lab_run3_", "arpg_lab_run4_",
                "aspen_run0_", "aspen_run1_", "aspen_run2_", "aspen_run3_", "aspen_run4_", "aspen_run5_", "aspen_run6_", "aspen_run7_", "aspen_run8_", "aspen_run9_", "aspen_run10_", "aspen_run11_",
                "ec_hallways_run0_", "ec_hallways_run1_", "ec_hallways_run2_", "ec_hallways_run3_", "ec_hallways_run4_", 
                "edgar_army_run0_", "edgar_army_run1_", "edgar_army_run2_", "edgar_army_run3_", "edgar_army_run4_", "edgar_army_run5_",
                "edgar_classroom_run0_", "edgar_classroom_run1_", "edgar_classroom_run2_", "edgar_classroom_run3_", "edgar_classroom_run4_", "edgar_classroom_run5_",
                "outdoors_run0_", "outdoors_run1_", "outdoors_run2_", "outdoors_run3_", "outdoors_run4_", "outdoors_run5_", "outdoors_run6_", "outdoors_run7_", "outdoors_run8_", "outdoors_run9_"
               ]

def calculate_resolution(points):
    # Compute the Euclidean distance between neighboring points
    kdtree = cKDTree(points)
    _, indices = kdtree.query(points, k=2)  # Find the nearest 2 points for each point
    distances = np.linalg.norm(points[indices[:, 1]] - points[indices[:, 0]], axis=1)

    # Calculate the average distance as the resolution
    resolution = np.mean(distances)
    #print("Resolution:",resolution, np.min(distances), np.max(distances))

    return resolution

def get_anchor_points(pcd, seed_point, num_points):
    # Create a KDTree from the mesh vertices
    kdtree = o3d.geometry.KDTreeFlann(pcd)
    [k, idx, _] = kdtree.search_knn_vector_3d(seed_point, num_points)
    patch_points = np.asarray(pcd.points)[idx, :]
    patch_pcd = o3d.geometry.PointCloud()
    patch_pcd.points = o3d.utility.Vector3dVector(patch_points)
    pcd_fps = patch_pcd.farthest_point_down_sample(4)

    anchor_points = [seed_point]
    for pt in pcd_fps.points:
        anchor_points.append(pt)
    return anchor_points

def geodesic_growth(lidar_pcd, radar_pcd, seed_point, num_points, ratio):
    # Create a KDTree from the mesh vertices
    points = np.array(lidar_pcd.points)
    points_2d = points[:,:2]
    kdtree = KDTree(points_2d)
    [k, idx] = kdtree.query(seed_point[:2], num_points)
    
    lidar_patch_points = np.asarray(lidar_pcd.points)[idx, :]
    lidar_patch_pcd = o3d.geometry.PointCloud()
    lidar_patch_pcd.points = o3d.utility.Vector3dVector(lidar_patch_points)
    
    # Create a KDTree from the mesh vertices
    points = np.array(radar_pcd.points)
    points_2d = points[:,:2]
    kdtree = KDTree(points_2d)
    [k, idx] = kdtree.query(seed_point[:2], int(num_points*ratio))
    radar_patch_points = np.asarray(radar_pcd.points)[idx, :]
    radar_patch_pcd = o3d.geometry.PointCloud()
    radar_patch_pcd.points = o3d.utility.Vector3dVector(radar_patch_points)
        

    return lidar_patch_pcd, radar_patch_pcd

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
        
    # pcd_dir = "/home/ajay/catkin_radar/coloradar/octomaps/pcd"
    # gt_dir = "/home/ajay/ARPG/RMap/data/ColoRadar/8K/gt_8000"
    # input_dir = "/home/ajay/ARPG/RMap/data/ColoRadar/8K/input_2000"

    files = os.listdir(args.pcd_dir)
    lidar_files = sorted([os.path.join(args.pcd_dir, f) for f in files if f.endswith("lidar_filtered.ply")])
    radar_files = sorted([os.path.join(args.pcd_dir, f) for f in files if f.endswith("radar.ply")])

    import shutil
    shutil.rmtree(args.gt_dir)
    shutil.rmtree(args.input_dir)
    
    if not os.path.exists(args.gt_dir):
        os.makedirs(args.gt_dir)
    

    if not os.path.exists(args.input_dir):
        os.makedirs(args.input_dir)
    
    # datatset_gt_files= os.listdir("/home/ajay/ColoRadar/ColoRadar/gt")
    for f in radar_files:
        radar_pcd = o3d.io.read_point_cloud(f)
        lidar_filename = f.replace("radar.ply", "lidar_filtered.ply")
        lidar_pcd = o3d.io.read_point_cloud(lidar_filename)
        scene_name = os.path.basename(f).replace("radar.ply","")
        #pose_files = [f for f in datatset_gt_files if scene_name in f]
        # num_seeds = len(pose_files)
        num_seeds = int(len(lidar_pcd.points)*12.5/num_points)
        print(scene_name, num_seeds, len(lidar_pcd.points))
        
        seed_indices = np.random.choice(np.asarray(radar_pcd.points).shape[0], num_seeds, replace=False)
        seed_points = np.asarray(radar_pcd.points)[seed_indices]
        for i in range(seed_points.shape[0]):
            #print(i, seed_points[i])
            #print(input_filepath, gt_filepath)
            #print(i, lidar_seed_points[i], radar_seed_points[i])
            lidar, radar = geodesic_growth(lidar_pcd, radar_pcd, seed_points[i], num_points, ratio)
            gt_filename =  f"{os.path.basename(lidar_filename).split('.')[0]}_{i:04d}.pcd"
            gt_filepath = os.path.join(args.gt_dir, gt_filename)
            gt_filepath = gt_filepath.replace("_lidar_filtered","")
            input_filename =  f"{os.path.basename(f).split('.')[0]}_{i:04d}.pcd"
            input_filepath = os.path.join(args.input_dir, input_filename)
            input_filepath = input_filepath.replace("_radar","")
    
            o3d.io.write_point_cloud(gt_filepath, lidar, True, True)
            #print(gt_filepath)
            o3d.io.write_point_cloud(input_filepath, radar, True, True)
        

