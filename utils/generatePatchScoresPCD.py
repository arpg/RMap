# Combine prediction from model to generate the entire map and assess the performance

import os
import sys  
import time
from datetime import datetime
import open3d as o3d
import math
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
import numpy as np
from extensions.chamfer_dist import ChamferDistanceL1, ChamferDistanceL2, ChamferDeviationL2
import torch

gt_dir = "UpPoinTr/data/ColoRadar/gt"
pred_dir = "output/UpPoinTr_FPSRadarLarge/"
lidar_dir = "gt_maps"
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

voxel_size = 0.15
files = sorted(os.listdir(pred_dir))
print(len(files))
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

down_L1 = []
down_L2 = []
down_fscore = []
down_avg_d = []

if __name__ == "__main__":
    CD_L2 = ChamferDistanceL2(ignore_zeros=True)
    CD_L1 = ChamferDistanceL1(ignore_zeros=True)
    CD_L2Dev = ChamferDeviationL2(ignore_zeros=True)
    
    scene_L1 = []
    scene_L2 = []
    scene_fscore = []
    for scene in scene_names:
        
        points = []
        scene_pcd = o3d.geometry.PointCloud()
        pcd_files = [f for f in files if f.endswith("_fine.pcd") and scene in f]
        gt_files = [os.path.join(gt_dir, f.split("_fine")[0]+".pcd") for f in pcd_files]
        pcd_files = [os.path.join(pred_dir, f) for f in pcd_files]
        
        print(scene, len(pcd_files), len(gt_files))
        if(len(pcd_files)==0):
            continue
        
        for i,f in enumerate(pcd_files):
            
            pcd = o3d.io.read_point_cloud(pcd_files[i])
            gt_pcd = o3d.io.read_point_cloud(gt_files[i])
            pcd_points = np.array(pcd.points)
            gt_points = np.array(gt_pcd.points)
            centroid = pcd_points[0]
            pcd_points = pcd_points - centroid
            pcd_points = pcd_points/25.0
            pcd_points = np.clip(pcd_points, -1, 1)
            gt_points = gt_points - centroid
            gt_points = gt_points/25.0
            gt_points = np.clip(gt_points, -1, 1)
            #print(len(pcd.points), len(gt_pcd.points))
            input = torch.from_numpy(np.array(pcd_points)).float().unsqueeze(0).cuda()
            gt = torch.from_numpy(np.array(gt_points)).float().unsqueeze(0).cuda()
            scene_L1.append(CD_L1(input, gt).detach().cpu().numpy())
            scene_L2.append(CD_L2(input, gt).detach().cpu().numpy())
            res = get_fscore(pcd, gt_pcd)
            scene_fscore.append(res)
         
        #print("Patch scores for ", scene, np.mean(scene_L1), np.mean(scene_L2), np.mean(scene_fscore))
        down_scene_pcd = o3d.io.read_point_cloud(os.path.join(pred_dir, scene+"recon.pcd"))
        #print(len(scene_pcd.points))
        down_scene_pcd = down_scene_pcd.voxel_down_sample(voxel_size=voxel_size)
        print("Final Combined Scene points", len(down_scene_pcd.points))
        #o3d.io.write_point_cloud(os.path.join(pred_dir, scene+"recon.pcd"), scene_pcd, True, True)
        # if scene == "aspen_run1_":
        #     scene = "aspen_run1"
        pcd = o3d.io.read_point_cloud(os.path.join(lidar_dir, scene+"lidar_filtered.ply"))
        #print("Lidar Scene Points", len(pcd.points))
        pts_count = pts_count + len(pcd.points) - len(down_scene_pcd.points)
        d = pcd.compute_point_cloud_distance(down_scene_pcd)
        #print(np.sum(d))
        input = torch.from_numpy(np.array(down_scene_pcd.points)).float().unsqueeze(0).cuda()
        gt = torch.from_numpy(np.array(pcd.points)).float().unsqueeze(0).cuda()
        ret = CD_L2Dev(input, gt).detach().cpu().numpy()
        down_L1.append(CD_L1(input, gt).detach().cpu().numpy())
        down_L2.append(CD_L2(input, gt).detach().cpu().numpy())
        res = get_fscore(down_scene_pcd, pcd)
        down_fscore.append(res)



        print(f"Recon Scene: {scene} CD_L1: {down_L1[-1]} CD_L2: {down_L2[-1]}, F-Score: {res}, Dist: {ret}")
        q = np.array([50,75,90,95])

        d = pcd.compute_point_cloud_distance(down_scene_pcd)
        down_avg_d.extend(d)   
        print(f"  Scene: {scene} Noise: {np.percentile(d, q=q)}")
        print("Patch scores for ", np.mean(scene_L1), np.mean(scene_L2), np.mean(scene_fscore))







print("Overall Reconstruction")
print(np.mean(down_L1), np.mean(down_L2), np.mean(down_fscore))
q = np.array([50,75,90,95])
print("Avg DD: ",np.percentile(down_avg_d,q=q))
print("Total missed points", pts_count)
print("Patch scores for ", np.mean(scene_L1), np.mean(scene_L2), np.mean(scene_fscore))

