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
                "arpg_lab_run0_", "arpg_lab_run1_", "arpg_lab_run2_", "arpg_lab_run3_", "arpg_lab_run4_",
                "aspen_run0_", "aspen_run1_", "aspen_run2_", "aspen_run3_", "aspen_run4_", "aspen_run5_", "aspen_run6_", "aspen_run7_", "aspen_run8_", "aspen_run9_", "aspen_run10_", "aspen_run11_",
                "ec_hallways_run0_", "ec_hallways_run1_", "ec_hallways_run2_", "ec_hallways_run3_", "ec_hallways_run4_", 
                "edgar_army_run0_", "edgar_army_run1_", "edgar_army_run2_", "edgar_army_run3_", "edgar_army_run4_", "edgar_army_run5_",
                "edgar_classroom_run0_", "edgar_classroom_run1_", "edgar_classroom_run2_", "edgar_classroom_run3_", "edgar_classroom_run4_", "edgar_classroom_run5_",
                "outdoors_run0_", "outdoors_run1_", "outdoors_run2_", "outdoors_run3_", "outdoors_run4_", "outdoors_run5_", "outdoors_run6_", "outdoors_run7_", "outdoors_run8_", "outdoors_run9_"
               ]
voxel_size = 0.15


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd_dir', type = str, help = 'Base folder contatning entire lidar and radar pointclouds')
    parser.add_argument('--pred_dir', type = str, help = 'Folder containing scene reconstructed pointclouds')
    args = parser.parse_args()
       
    CD_L2 = ChamferDistanceL2(ignore_zeros=True)
    CD_L1 = ChamferDistanceL1(ignore_zeros=True)
    q = np.array([50,75,90,95])
    for scene in scene_names:
        if "run0" not in scene:
            continue
        radar_pcd = o3d.io.read_point_cloud(os.path.join(args.pcd_dir, scene+"radar.ply"))

        pcd = o3d.io.read_point_cloud(os.path.join(args.pcd_dir, scene+"lidar_filtered.ply"))
        d = pcd.compute_point_cloud_distance(radar_pcd)
        
        print(f" Input Scene: {scene} Noise: {np.percentile(d, q=q)}")

        pred_pcd = o3d.io.read_point_cloud(os.path.join(args.pred_dir, scene+"recon.pcd"))
        d = pcd.compute_point_cloud_distance(pred_pcd)
        print(f" Pred Scene: {scene} Noise: {np.percentile(d, q=q)}")

