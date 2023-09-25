import os
import sys
import time
from datetime import datetime
import open3d as o3d
import math
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
import numpy as np
from extensions.chamfer_dist import ChamferDistanceL1, ChamferDistanceL2
import torch
import argparse

scene_names = [
                "arpg_lab_run0", "arpg_lab_run1", "arpg_lab_run2", "arpg_lab_run3", "arpg_lab_run4",
                "aspen_run0", "aspen_run1", "aspen_run2", "aspen_run3", "aspen_run4", "aspen_run5", "aspen_run6", "aspen_run7", "aspen_run8", "aspen_run9", "aspen_run10", "aspen_run11",
                "ec_hallways_run0", "ec_hallways_run1", "ec_hallways_run2", "ec_hallways_run3", "ec_hallways_run4", 
                "edgar_army_run0", "edgar_army_run1", "edgar_army_run2", "edgar_army_run3", "edgar_army_run4", "edgar_army_run5",
                "edgar_classroom_run0", "edgar_classroom_run1", "edgar_classroom_run2", "edgar_classroom_run3", "edgar_classroom_run4", "edgar_classroom_run5",
                "outdoors_run0", "outdoors_run1", "outdoors_run2", "outdoors_run3", "outdoors_run4", "outdoors_run5", "outdoors_run6", "outdoors_run7", "outdoors_run8", "outdoors_run9"
               ]
voxel_size = 0.15


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd_dir', type = str, help = 'Base folder contatning entire lidar and radar pointclouds')
    parser.add_argument('--pred_dir', type = str, help = 'Folder containing predicted pointclouds')
    args = parser.parse_args()
    files = os.listdir(args.pred_dir)
    
    CD_L2 = ChamferDistanceL2(ignore_zeros=True)
    CD_L1 = ChamferDistanceL1(ignore_zeros=True)
    for scene in scene_names:

        points = []
        scene_pcd = o3d.geometry.PointCloud()
        pcd_files = [os.path.join(args.pred_dir, f) for f in files if f.endswith(".pcd") and scene in f]
        print(scene, len(pcd_files))
        for i,f in enumerate(pcd_files):
            
            pcd = o3d.io.read_point_cloud(f)
            points.extend(pcd.points)
            if i %100 == 1:
                scene_pcd.points = o3d.utility.Vector3dVector(points)
                scene_pcd = scene_pcd.voxel_down_sample(voxel_size=voxel_size)
                points = scene_pcd.points
        
        scene_pcd.points = o3d.utility.Vector3dVector(points)
        scene_pcd = scene_pcd.voxel_down_sample(voxel_size=voxel_size)
        o3d.io.write_point_cloud(os.path.join(args.pred_dir, scene+"_recon.pcd"), scene_pcd, True, True)
        pcd = o3d.io.read_point_cloud(os.path.join(args.pcd_dir, scene+"_lidar_filtered.ply"))
        d = pcd.compute_point_cloud_distance(scene_pcd)
        input = torch.from_numpy(np.array(scene_pcd.points))
        gt = torch.from_numpy(np.array(pcd.points))
        
        print(f"Scene: {scene} CD_L1: {CD_L1(input, gt)} CD_L2: {CD_L2(input, gt)}")

        