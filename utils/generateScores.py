import os
import sys  
import time
from datetime import datetime
import open3d as o3d
import math
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
import numpy as np
from UpPoinTr.extensions.chamfer_dist import ChamferDistanceL1, ChamferDistanceL2, ChamferDeviationL2
import torch

gt_dir = "UpPoinTr/data/CooloRadar/gt"
pcd_dir = "output/AdaPoinTr_FPSRadarLarge/"
maps_dir = "gt_maps"
scene_names = [
                "arpg_lab_run0_", "arpg_lab_run1_",
                "aspen_run0_", "aspen_run1_", "aspen_run2_", "aspen_run3_", "aspen_run4_", "aspen_run5_",
                "ec_hallways_run0_", "ec_hallways_run1_",
                "edgar_army_run0_", "edgar_army_run1_", "edgar_army_run2_",
                "edgar_classroom_run0_", "edgar_classroom_run1_", "edgar_classroom_run2_",
                "outdoors_run0_", "outdoors_run1_", "outdoors_run2_", "outdoors_run3_", "outdoors_run4_"
               ]

voxel_size = 0.15
files = sorted(os.listdir(pcd_dir))
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


if __name__ == "__main__":    
    pred_L1 = []
    pred_L2 = []
    pred_fscore = []
    pred_avg_dev = []

    input_L1 = []
    input_L2 = []
    input_fscore = []
    input_avg_dev = []

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
        pcd_files = [os.path.join(pcd_dir, f) for f in pcd_files]
        if(len(pcd_files)==0):
            continue
        
        for i,f in enumerate(pcd_files):    
            # Calculate Patch Scores
            pcd = o3d.io.read_point_cloud(pcd_files[i])
            gt_pcd = o3d.io.read_point_cloud(gt_files[i])
            pcd_points = np.array(pcd.points)
            gt_points = np.array(gt_pcd.points)
            # Normalize points for Patch-level comparision
            centroid = pcd_points[0]
            pcd_points = pcd_points - centroid
            pcd_points = pcd_points/25.0
            pcd_points = np.clip(pcd_points, -1, 1)
            gt_points = gt_points - centroid
            gt_points = gt_points/25.0
            gt_points = np.clip(gt_points, -1, 1)
            
            input = torch.from_numpy(np.array(pcd_points)).float().unsqueeze(0).cuda()
            gt = torch.from_numpy(np.array(gt_points)).float().unsqueeze(0).cuda()
            scene_L1.append(CD_L1(input, gt).detach().cpu().numpy())
            scene_L2.append(CD_L2(input, gt).detach().cpu().numpy())
            res = get_fscore(pcd, gt_pcd)
            scene_fscore.append(res)

            
        down_scene_pcd = o3d.io.read_point_cloud(os.path.join(pcd_dir, scene+"recon.pcd"))
        down_scene_pcd = down_scene_pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd = o3d.io.read_point_cloud(os.path.join(maps_dir, scene+"lidar_filtered.ply"))
        d = pcd.compute_point_cloud_distance(down_scene_pcd)

        # Calculate scores for reconstructed pcd
        input = torch.from_numpy(np.array(down_scene_pcd.points)).float().unsqueeze(0).cuda()
        gt = torch.from_numpy(np.array(pcd.points)).float().unsqueeze(0).cuda()
        ret = CD_L2Dev(input, gt).detach().cpu().numpy()
        pred_L1.append(CD_L1(input, gt).detach().cpu().numpy())
        pred_L2.append(CD_L2(input, gt).detach().cpu().numpy())
        res = get_fscore(down_scene_pcd, pcd)
        pred_fscore.append(res)
        print(f"Recon Scene: {scene} CD_L1: {pred_L1[-1]} CD_L2: {pred_L2[-1]}, F-Score: {res}, Dist: {ret}")
        q = np.array([50,75,90,95])
        d = pcd.compute_point_cloud_distance(down_scene_pcd)
        pred_avg_dev.extend(d)   
        print(f"  Scene: {scene} Noise: {np.percentile(d, q=q)}")
        print("Patch scores for ", np.mean(scene_L1), np.mean(scene_L2), np.mean(scene_fscore))

        #### Compare with Input Scene
        radar_pcd = o3d.io.read_point_cloud(os.path.join(maps_dir, scene+"radar.ply"))
        input = torch.from_numpy(np.array(radar_pcd.points)).float().unsqueeze(0).cuda()
        
        input_L1.append(CD_L1(input, gt).detach().cpu().numpy())
        input_L2.append(CD_L2(input, gt).detach().cpu().numpy())
        ret = CD_L2Dev(input, gt).detach().cpu().numpy()
        res = get_fscore(radar_pcd, pcd)
        input_fscore.append(res)

        print(f"Radar Scene: {scene} CD_L1: {input_L1[-1]} CD_L2: {input_L2[-1]}, F-Score: {res}, Dist: {ret}")
        q = np.array([50,75,90,95])
        d = pcd.compute_point_cloud_distance(radar_pcd)
        input_avg_dev.extend(d)
        print(f"Radar  Scene: {scene} Noise: {np.percentile(d, q=q)}")


print("Overall Input")
print(f"CD_L1: {np.mean(input_L1)}, CD_L2:{np.mean(input_L2)}, F-SCORE: {np.mean(input_fscore)}")
q = np.array([50,75,90,95])
print("Input Avg DD: ",np.percentile(input_avg_dev,q=q))

print("Overall Reconstruction")
print(f"CD_L1: {np.mean(pred_L1)}, CD_L2:{np.mean(pred_L2)}, F-SCORE: {np.mean(pred_fscore)}")
q = np.array([50,75,90,95])
print("Avg DD: ",np.percentile(pred_avg_dev,q=q))
print(f"Patch scores for  CD_L1: {np.mean(scene_L1)}, CD_L2: {np.mean(scene_L2)}, F-SCORE: {np.mean(scene_fscore)}")

