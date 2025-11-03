import os
import sys  
import time
from datetime import datetime
import open3d as o3d
import math
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
import numpy as np
from UpPoinTr.extensions.chamfer_dist import ChamferDistanceL1, ChamferDistanceL2
import torch

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
    
    L1 = []
    L2 = []
    fscore = []
    avg_d = []

    CD_L2 = ChamferDistanceL2(ignore_zeros=True)
    CD_L1 = ChamferDistanceL1(ignore_zeros=True)
    for scene in scene_names:

        points = []
        scene_pcd = o3d.geometry.PointCloud()
        pcd_files = [os.path.join(pcd_dir, f) for f in files if f.endswith(".pcd") and scene in f]
        print(scene, len(pcd_files))
        if(len(pcd_files)==0):
            continue
        
        for i,f in enumerate(pcd_files):    
            pcd = o3d.io.read_point_cloud(f)
            centroid = pcd.points[0]
            pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            points.extend(pcd.points)
        
            if i %100 == 1:
                scene_pcd.points = o3d.utility.Vector3dVector(points)
                scene_pcd = scene_pcd.voxel_down_sample(voxel_size=voxel_size)
                points = scene_pcd.points
        
        scene_pcd.points = o3d.utility.Vector3dVector(np.unique(points, axis=0))
        
        scene_pcd = scene_pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd = o3d.io.read_point_cloud(os.path.join(maps_dir, scene+"lidar_filtered.ply"))
        d = pcd.compute_point_cloud_distance(scene_pcd)
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
print(f"CD_L1: {np.mean(L1)}, CD_L2:{np.mean(L2)}, F-SCORE: {np.mean(fscore)}")
q = np.array([50,75,90,95])
print("Avg DD: ",np.percentile(avg_d,q=q))
