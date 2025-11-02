# Compare the initial radar map and the reconstructed map(from model outputs)
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
input_dev = []

L1 = []
L2 = []
fscore = []
avg_d = []
dev = []


if __name__ == "__main__":
    CD_L2 = ChamferDistanceL2(ignore_zeros=True)
    CD_L1 = ChamferDistanceL1(ignore_zeros=True)
    CD_L2Dev = ChamferDeviationL2(ignore_zeros=True)
    
    for scene in scene_names:
        
         
        #print("Patch scores for ", scene, np.mean(scene_L1), np.mean(scene_L2), np.mean(scene_fscore))
        scene_pcd = o3d.io.read_point_cloud(os.path.join(pred_dir, scene+"recon.pcd"))
        print("Final Combined Scene points", len(scene_pcd.points))
        pcd = o3d.io.read_point_cloud(os.path.join(lidar_dir, scene+"lidar_filtered.ply"))
        #print("Lidar Scene Points", len(pcd.points))
        pts_count = pts_count + len(pcd.points) - len(scene_pcd.points)
        d = pcd.compute_point_cloud_distance(scene_pcd)
        #print(np.sum(d))
        input = torch.from_numpy(np.array(scene_pcd.points)).float().unsqueeze(0).cuda()
        gt = torch.from_numpy(np.array(pcd.points)).float().unsqueeze(0).cuda()
        ret = CD_L2Dev(input, gt).detach().cpu().numpy()
        dev.append(ret[1:])
        L1.append(CD_L1(input, gt).detach().cpu().numpy())
        L2.append(CD_L2(input, gt).detach().cpu().numpy())
        res = get_fscore(scene_pcd, pcd)
        fscore.append(res)



        print(f"Recon Scene: {scene} CD_L1: {L1[-1]} CD_L2: {L2[-1]}, F-Score: {res}, Dist: {ret}")
        q = np.array([50,75,90,95])

        d = pcd.compute_point_cloud_distance(scene_pcd)
        avg_d.extend(d)   
        print(f"  Scene: {scene} Noise: {np.percentile(d, q=q)}")










        #### Input Scene
        radar_pcd = o3d.io.read_point_cloud(os.path.join(lidar_dir, scene+"radar.ply"))
        input = torch.from_numpy(np.array(radar_pcd.points)).float().unsqueeze(0).cuda()
        
        input_L1.append(CD_L1(input, gt).detach().cpu().numpy())
        input_L2.append(CD_L2(input, gt).detach().cpu().numpy())
        ret = CD_L2Dev(input, gt).detach().cpu().numpy()
        input_dev.append(ret[1:])
        res = get_fscore(radar_pcd, pcd)
        input_fscore.append(res)

        print(f"Radar Scene: {scene} CD_L1: {input_L1[-1]} CD_L2: {input_L2[-1]}, F-Score: {res}, Dist: {ret}")
        q = np.array([50,75,90,95])

        d = pcd.compute_point_cloud_distance(radar_pcd)
        input_avg_d.extend(d)
        print(f"Radar  Scene: {scene} Noise: {np.percentile(d, q=q)}")
        


print("Overall Input")
print(np.mean(input_L1), np.mean(input_L2), np.mean(input_fscore), np.mean(input_dev, axis=0))
q = np.array([50,75,90,95])
print("Input Avg DD: ",np.percentile(input_avg_d,q=q))

print("Overall Reconstruction")
print(np.mean(L1), np.mean(L2), np.mean(fscore), np.mean(dev, axis=0))
q = np.array([50,75,90,95])
print("Avg DD: ",np.percentile(avg_d,q=q))
print("Total missed points", pts_count)
#print("Patch scores for ", np.mean(scene_L1), np.mean(scene_L2), np.mean(scene_fscore))

