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


def get_seed_points(filepath):
    points = np.loadtxt(filepath, delimiter=',')
    points = points[:,:2]
    seed_points = [points[0]]
    for i in range(1, points.shape[0]):
        includeFlag = True
        for pt in seed_points:
            d = np.linalg.norm(pt - points[i])
            if(d<0.9):
                includeFlag = False
                break
        if(includeFlag):
            seed_points.append(points[i])
    seed_points = np.array(seed_points)
    print(seed_points.shape, points.shape, os.path.basename(filepath))
    x = seed_points[:,0]
    y = seed_points[:,1]
    markers_on = range(0,len(x),2)
    fig, ax = plt.subplots()
    ax.plot(x, y, '-D', markevery=markers_on)
    fig.savefig(filepath.replace(".txt", "_odom_xy.png"))
    return seed_points

if __name__ == "__main__":
    
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--poses_dir', type = str, help = 'Base folder contatning entire lidar and radar pointclouds')
    args = parser.parse_args()
    
    files = os.listdir(args.poses_dir)
    
    for f in files:
        if f.endswith(".txt"):
            seed_points = get_seed_points(os.path.join(args.poses_dir, f))
        

