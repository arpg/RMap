import os
import sys
import time
from datetime import datetime
import open3d as o3d
import math
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
import numpy as np
import matplotlib.pyplot as plt
import copy 
import argparse
from scipy.interpolate import make_interp_spline



def estimateNoise(input, target):
    kdtree = o3d.geometry.KDTreeFlann(target)
    diff = []
    for pt in input.points:
        [k, idx, _] = kdtree.search_knn_vector_3d(pt, 1)
        target_pt = np.asarray(target.points)[idx, :]
        diff.append(np.linalg.norm(np.array(pt) - target_pt))
        # diff.append(np.sum(np.abs(np.array(pt) - target_pt)))
        
        #print(diff[-1])
    return np.array(diff)

    
       
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--pcd_dir', type = str, help = 'Base dir of complete radar and lidar pointclouds')
    args = parser.parse_args()
    pcd_dir = args.pcd_dir

    files = os.listdir(pcd_dir)
    lidar_files = sorted([os.path.join(pcd_dir, f) for f in files if f.endswith("lidar_filtered.ply")])
    radar_files = sorted([os.path.join(pcd_dir, f) for f in files if f.endswith("radar.ply")])
    final_dist = []
    q = np.array([50,75,90,95])
    for f in radar_files:
        # if not "outdoors" in f:
        #     continue
        radar_pcd = o3d.io.read_point_cloud(f)
        lidar_filename = f.replace("radar.ply", "lidar_filtered.ply")
        lidar_pcd = o3d.io.read_point_cloud(lidar_filename)
        # dist = estimateNoise(radar_pcd,lidar_pcd)
        dist = lidar_pcd.compute_point_cloud_distance(radar_pcd)
        fig, ax = plt.subplots()
        dist = np.array(dist)
        final_dist.extend(dist)
        bins=np.linspace(0,max(dist), int(np.floor(max(dist)/0.15))+2)
        ax.hist(dist, bins=bins)
        fig.savefig(f.replace("radar.ply","noise.png"))
        print(os.path.basename(f), len(radar_pcd.points), len(lidar_pcd.points), np.mean(dist, axis=0), np.median(dist,axis=0), np.percentile(dist, q=q), np.max(dist), dist.shape[0])
    fig, ax = plt.subplots()
    final_dist = np.array(final_dist)
    print(np.mean(final_dist, axis=0), np.median(final_dist,axis=0), np.percentile(final_dist, q=q), np.max(final_dist), final_dist.shape[0])
    
    # bins=np.linspace(0,max(final_dist), int(np.floor(max(final_dist)/0.15))+2)
    bins = np.linspace(0, max(final_dist), 100)
    hist, bin_edges = np.histogram(final_dist, bins)
    bin_centers = 0.5*(bin_edges[:-1]+bin_edges[1:])
    
    # hist = hist[1:]
    # bin_centers = bin_centers[1:]
    hist_spline = make_interp_spline(bin_centers, hist )
    smoothened_bins = np.linspace(bin_centers[0], bin_centers[-1], 500)
    smoothened_hist = hist_spline(smoothened_bins)
    ax.plot(smoothened_bins, smoothened_hist)
    ax.set_xlabel('Deviation from closest point in GT', 
               fontweight ='bold')
    
    ax.set_title('Noise distribution', fontsize = 14, fontweight ='bold')
    fig.savefig(os.path.join(pcd_dir, "noise.png"))
    
    fig, ax = plt.subplots()
    ax.plot(bin_centers, hist)
    ax.set_xlabel('Deviation from closest point in GT', 
               fontweight ='bold')
    
    ax.set_title('Noise distribution', fontsize = 14, fontweight ='bold')
    fig.savefig(os.path.join(pcd_dir, "noise_original.png"))
    