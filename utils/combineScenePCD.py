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

# pcd_dir = "/home/ajay/ARPG/RMap/data/EncoderFeatureSampling"
# pcd_dir = "/home/ajay/ARPG/RMap/RPoinTr/V100/SPoinTr_V100_AreaSampling_Pred_Normalize/"
# pcd_dir = "/home/ajay/ARPG/IROS/Pred_UpPoinTr_PCNEncoderD4DecoderD6_Heads4Dim256_OnlyAttnGraph_54Split_BS16_Factors1_4_4_Var50InputCenter"
#pcd_dir = "/media/giantdrive/kharlow/coloradar/RMap/PoinTr/ColoRadar/FPSRadar2/gt"
pcd_dir = "/home/ajay/RMap/AdaPoinTr_FPSRadarLarge/"
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
# def postprocess(pcd):
#     kdtree = o3d.geometry.KDTreeFlann(pcd)
#     seed_point = [0,0,0]
#     [k, idx, _] = kdtree.search_knn_vector_xd(seed_point, len(pcd.points))
#     patch_points = np.asarray(points)[idx[::3], :]
#     patch_pcd = o3d.geometry.PointCloud()
#     patch_pcd.points = o3d.utility.Vector3dVector(patch_points)
#     return patch_pcd

def postprocess(pcd):
    # print("Radar Input", np.array(pcd.points).shape)
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=64,std_ratio=2)
    # filtered_pcd = o3d.geometry.PointCloud()
    # filtered_pcd.points = o3d.utility.Vector3dVector(pcd.points[ind,:])
    # print("Radar filtering", np.array(pcd.points).shape)    
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

        points = []
        scene_pcd = o3d.geometry.PointCloud()
        pcd_files = [os.path.join(pcd_dir, f) for f in files if f.endswith(".pcd") and scene in f]
        print(scene, len(pcd_files))
        if(len(pcd_files)==0):
            continue
        
        for i,f in enumerate(pcd_files):
            
            pcd = o3d.io.read_point_cloud(f)
            centroid = pcd.points[0]
            # pcd = pcd.translate(-centroid)
            # print(pcd.get_center(), centroid)
            pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            # pcd = pcd.translate(centroid)
            points.extend(pcd.points)
            # pcd_points = np.load(f)
            # print(pcd_points.shape)
            # points.extend(pcd_points)

            if i %100 == 1:
                scene_pcd.points = o3d.utility.Vector3dVector(points)
                scene_pcd = scene_pcd.voxel_down_sample(voxel_size=voxel_size)
                points = scene_pcd.points
        
        scene_pcd.points = o3d.utility.Vector3dVector(np.unique(points, axis=0))
        print(len(scene_pcd.points))
        # scene_pcd, ind = scene_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=2.0)
        
        print(len(scene_pcd.points))
        scene_pcd = scene_pcd.voxel_down_sample(voxel_size=voxel_size)
        print("Final Combined Scene points", len(scene_pcd.points))
        #o3d.io.write_point_cloud(os.path.join(pcd_dir, scene+"recon.pcd"), scene_pcd, True, True)
        # if scene == "aspen_run1_":
        #     scene = "aspen_run1"
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
        """


        #### Input Scene
        radar_pcd = o3d.io.read_point_cloud(os.path.join(lidar_dir, scene+"radar.ply"))
        input = torch.from_numpy(np.array(radar_pcd.points)).float().unsqueeze(0).cuda()
        
        input_L1.append(CD_L1(input, gt).detach().cpu().numpy())
        input_L2.append(CD_L2(input, gt).detach().cpu().numpy())
        res = get_fscore(radar_pcd, pcd)
        input_fscore.append(res)

        print(f"Radar Scene: {scene} CD_L1: {input_L1[-1]} CD_L2: {input_L2[-1]}, F-Score: {res}")
        q = np.array([50,75,90,95])

        d = pcd.compute_point_cloud_distance(radar_pcd)
        input_avg_d.extend(d)
        print(f"Radar  Scene: {scene} Noise: {np.percentile(d, q=q)}")
        

        # radar_pcd = o3d.io.read_point_cloud(os.path.join(lidar_dir, scene+"radar.ply"))
        # input = torch.from_numpy(np.array(radar_pcd.points)).float().unsqueeze(0).cuda()
        # print(f"Radar Scene: {scene} CD_L1: {CD_L1(input, gt)} CD_L2: {CD_L2(input, gt)}")
        filtered_pcd = postprocess(scene_pcd)
        print("Filtered Combined Scene points", len(filtered_pcd.points))
        # o3d.io.write_point_cloud(os.path.join(pcd_dir, scene+"recon_filtered.pcd"), filtered_pcd, True, True)
        # # if scene == "aspen_run1_":
        # #     scene = "aspen_run1"
        # d = pcd.compute_point_cloud_distance(filtered_pcd)
        # print(np.sum(d))
        input = torch.from_numpy(np.array(filtered_pcd.points)).float().unsqueeze(0).cuda()
        gt = torch.from_numpy(np.array(pcd.points)).float().unsqueeze(0).cuda()
        filtered_L1.append(CD_L1(input, gt).detach().cpu().numpy())
        filtered_L2.append(CD_L2(input, gt).detach().cpu().numpy())
        res = get_fscore(filtered_pcd, pcd)
        filtered_fscore.append(res)



        print(f"Filtered Scene: {scene} CD_L1: {L1[-1]} CD_L2: {L2[-1]}, F-Score: {res}")
        d = pcd.compute_point_cloud_distance(filtered_pcd)
        filtered_avg_d.extend(d)
        print(f"Filtered  Scene: {scene} Noise: {np.percentile(d, q=q)}")
        """
        # print(f"Scene: {scene} CD_L1: {CD_L1(input, gt)} CD_L2: {CD_L2(input, gt)}")

"""
print("Overall Input")
print(np.mean(input_L1), np.mean(input_L2), np.mean(input_fscore))
q = np.array([50,75,90,95])
print("Input Avg DD: ",np.percentile(input_avg_d,q=q))
"""

print("Overall Reconstruction")
print(np.mean(L1), np.mean(L2), np.mean(fscore))
q = np.array([50,75,90,95])
print("Avg DD: ",np.percentile(avg_d,q=q))
print("Total missed points", pts_count)
"""
print("Overall Filtered")
print(np.mean(filtered_L1), np.mean(filtered_L2), np.mean(filtered_fscore))
q = np.array([50,75,90,95])
print("Filtered Avg DD: ",np.percentile(filtered_avg_d,q=q))
"""
