"""
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/arpg_lab_run0.bag is 120.35720753669739 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/arpg_lab_run1.bag is 118.6608612537384 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/aspen_run0.bag is 104.59965920448303 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/aspen_run1.bag is 109.29767680168152 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/aspen_run2.bag is 127.55637860298157 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/aspen_run3.bag is 147.28143429756165 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/aspen_run4.bag is 103.36267185211182 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/aspen_run5.bag is 109.7201280593872 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/ec_hallways_run0.bag is 104.67724466323853 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/ec_hallways_run1.bag is 179.91307020187378 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/edgar_army_run0.bag is 478.47402572631836 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/edgar_army_run1.bag is 236.75068187713623 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/edgar_army_run2.bag is 144.7181315422058 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/edgar_classroom_run0.bag is 187.9365758895874 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/edgar_classroom_run1.bag is 171.24220395088196 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/edgar_classroom_run2.bag is 197.16022443771362 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/outdoors_run0.bag is 110.47448682785034 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/outdoors_run1.bag is 122.84659767150879 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/outdoors_run2.bag is 131.20369410514832 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/outdoors_run3.bag is 133.14802932739258 seconds.
The duration of the ROS bag file /media/giantdrive/kharlow/coloradar/bags/outdoors_run4.bag is 109.40973424911499 seconds.
3248.7907180786133
"""
import os
import numpy as np
import open3d as o3d

scene_names = [
        "arpg_lab_run0.bag", "arpg_lab_run1.bag",
        "aspen_run0.bag", "aspen_run1.bag", "aspen_run2.bag", "aspen_run3.bag", "aspen_run4.bag", "aspen_run5.bag",
        "ec_hallways_run0.bag", "ec_hallways_run1.bag", 
        "edgar_army_run0.bag", "edgar_army_run1.bag", "edgar_army_run2.bag", 
        "edgar_classroom_run0.bag", "edgar_classroom_run1.bag", "edgar_classroom_run2.bag", 
        "outdoors_run0.bag", "outdoors_run1.bag", "outdoors_run2.bag", "outdoors_run3.bag", "outdoors_run4.bag"
        ]



import rosbag

def get_duration_of_rosbag(rosbag_file):
    with rosbag.Bag(rosbag_file, 'r') as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        duration = end_time - start_time
        return duration

if __name__ == "__main__":
    d = []
    for f in scene_names:
        rosbag_file = os.path.join("/media/giantdrive/kharlow/coloradar/bags/",f)
        duration = get_duration_of_rosbag(rosbag_file)
        print(f"The duration of the ROS bag file {rosbag_file} is {duration} seconds.")
        d.append(duration)
    print(np.sum(d))
