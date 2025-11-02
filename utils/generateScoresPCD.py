# Generate scores for the reconstructed final map(from the predictions of the model)
# Run combineScenePCD.py before running the script

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

pcd_dir = "output/AdaPoinTr_FPSRadarLarge/"
"""
scene_names = [
        "arpg_lab_run0_", "arpg_lab_run1_", "arpg_lab_run2_", "arpg_lab_run3_", "arpg_lab_run4_",
        "aspen_run0_", "aspen_run1_", "aspen_run2_", "aspen_run3_", "aspen_run4_", "aspen_run5_", "aspen_run6_", "aspen_run7_", "aspen_run8_", "aspen_run9_", "aspen_run10_", "aspen_run11_",
        "ec_hallways_run0_", "ec_hallways_run1_", "ec_hallways_run2_", "ec_hallways_run3_", "ec_hallways_run4_", 
        "edgar_army_run0_", "edgar_army_run1_", "edgar_army_run2_", "edgar_army_run3_", "edgar_army_run4_", "edgar_army_run5_",
        "edgar_classroom_run0_", "edgar_classroom_run1_", "edgar_classroom_run2_", "edgar_classroom_run3_", "edgar_classroom_run4_", "edgar_classroom_run5_",
        "outdoors_run0_", "outdoors_run1_", "outdoors_run2_", "outdoors_run3_", "outdoors_run4_", "outdoors_run5_", "outdoors_run6_", "outdoors_run7_", "outdoors_run8_", "outdoors_run9_"
        ]
"""
scene_names = [
                "arpg_lab_run0_", "arpg_lab_run1_",
                "aspen_run0_", "aspen_run1_", "aspen_run2_", "aspen_run3_", "aspen_run4_", "aspen_run5_",
                "ec_hallways_run0_", "ec_hallways_run1_",
                "edgar_army_run0_", "edgar_army_run1_", "edgar_army_run2_",
                "edgar_classroom_run0_", "edgar_classroom_run1_", "edgar_classroom_run2_",
                "outdoors_run0_", "outdoors_run1_", "outdoors_run2_", "outdoors_run3_", "outdoors_run4_"
               ]

# scene_names = [
#                 "arpg_lab_run0_", "arpg_lab_run1_"
#                ]

# scene_names = ["G_canteen_circle_bright_1_0_","G_atrium_linear_bright_2_0_", "G_corridor_linear_bright_11_0_", "G_corridor_linear_bright_13_0_", "G_corridor_linear_bright_18_0_", "G_hallway_linear_bright_1_0_", "G_hallway_linear_bright_3_0_", "G_stairwell_linear_bright_1_0_", "G_vicon_eight_bright_1_0_", "G_vicon_eight_bright_5_0_"]
voxel_size = 0.15
files = sorted(os.listdir(pcd_dir))
print(len(files))
lidar_dir = "gt_maps"
pts_count =0

def postprocess(pcd):
    # print("Radar Input", np.array(pcd.points).shape)
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=64,std_ratio=2)
    return pcd

def get_fscore(pred, gt):
    th = 0.15
    dist1 = pred.compute_point_cloud_distance(gt)
    dist2 = gt.compute_point_cloud_distance(pred)

    recall = float(sum(d < th for d in dist2)) / float(len(dist2))
    precision = float(sum(d < th for d in dist1)) / float(len(dist1))
    result = 2 * recall * precision / (recall + precision) if recall + precision else 0.
    return result


input_L1 = []
input_L2 = []
input_fscore = []
input_avg_d = []


L1 = []
L2 = []
fscore = []
avg_d = []

filtered_L1 = []
filtered_L2 = []
filtered_fscore = []
filtered_avg_d = []

if __name__ == "__main__":
    CD_L2 = ChamferDistanceL2(ignore_zeros=True)
    CD_L1 = ChamferDistanceL1(ignore_zeros=True)
    for scene in scene_names:
        scene_pcd = o3d.io.read_point_cloud(os.path.join(pcd_dir, scene+"recon.pcd"))
        print(len(scene_pcd.points))
        scene_pcd = scene_pcd.voxel_down_sample(voxel_size=voxel_size)
        print("Final Combined Scene points", len(scene_pcd.points))
        pcd = o3d.io.read_point_cloud(os.path.join(lidar_dir, scene+"lidar_filtered.ply"))
        print("Lidar Scene Points", len(pcd.points))
        pts_count = pts_count + len(pcd.points) - len(scene_pcd.points)
        d = pcd.compute_point_cloud_distance(scene_pcd)
        print(np.sum(d))
        input = torch.from_numpy(np.array(scene_pcd.points)).float().unsqueeze(0).cuda()
        gt = torch.from_numpy(np.array(pcd.points)).float().unsqueeze(0).cuda()
        L1.append(CD_L1(input, gt).detach().cpu().numpy())
        L2.append(CD_L2(input, gt).detach().cpu().numpy())
        res = get_fscore(scene_pcd, pcd)
        fscore.append(res)



        print(f"Recon Scene: {scene} CD_L1: {L1[-1]} CD_L2: {L2[-1]}, F-Score: {res}")
        q = np.array([50,75,90,95])

        d = pcd.compute_point_cloud_distance(scene_pcd)
        avg_d.extend(d)   
        print(f"  Scene: {scene} Noise: {np.percentile(d, q=q)}")

print("Overall Reconstruction")
print(np.mean(L1), np.mean(L2), np.mean(fscore))
q = np.array([50,75,90,95])
print("Avg DD: ",np.percentile(avg_d,q=q))
print("Total missed points", pts_count)
