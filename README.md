# RMap: Millimter-Wave Radar Mapping Through Volumetric Upsampling
[[arXiv]](https://arxiv.org/pdf/2310.13188.pdf) [[Video]](https://ajaymopidevi.github.io/ajay/projects/2_project/)

RMap (RadarMapping), a method to generate highprecision 3D maps using radar point clouds extracted from an mmWave sensor. 

We present an end-to-end pipeline for generating the 3D maps from radar point clouds and demonstrate how these maps can be leveraged to construct a 3D map resembling lidar-based maps through UpPoinTr.
![System Diagram](Paper/SystemDiagram.png)

### Usage:
1. For the coloRadar dataset, the maps generated using radar and lidar (also lidar_filtered - considering lidar measurements only in the range and FOV of radar). The maps are stored in data/ply. For the points along the trajectory, the data is stored in data/poses
2. From the maps and poses, generate radar input and lidar groundtruth patches by:
    ```
    python utils/poseSample.py --pcd_dir ./data/ply --input_dir <SAVE_INPUT_DIR> --gt_dir <SAVE_GT_DIR>
    ```
3. Train/Test the UpPoinTr network with the generated input (and gt) patches. More details are available in <a href="[./UpPoinTr](https://github.com/ajaymopidevi/UpPoinTr)">UpPoinTr</a> repo.
4. Combine the UpPoinTr predicted patches by
    ```
    python combinescenePCD.py --pcd_dir ./data/ply --pred_dir <PREDICTED_PATCHES_DIR>
    ```
This saves the final combined map for scene and also outputs the CD-L1 and CD-L2 metrics



### For generating radar maps on a new dataset:
1. Install <a href="https://github.com/1988kramer/octomap/tree/feature/intensity_map">octomap</a>
2. ROS package dependecies:
    - <a href="https://github.com/1988kramer/octomap_mapping/tree/feature/radar_image">ocotmap_mapping</a>
    - <a href="https://github.com/arpg/dca1000_device_msgs">dca1000_device_msgs</a>
    - <a href="https://github.com/Alphakyl/octomap_radar_analysis">octomap_radar_analysis</a>
3. Create a custom launch file similar to ocotomap_radar_analysis/launch/ocotmap_mapping.launch file


### Results:
RMap genrated maps for ColoRadar dataset:
![Predicted](Paper/ResultsMaps.png)

Through this crosssection analysis, we see that the original radar map consists primarily of noise. However, the RMap generated map has a similar structure to the lidar map, distinguishing between free space and occupied space.
![Navigable](Paper/Analysis.png)


## Citation
If you find our work useful in your research, please consider citing: 
```
@article{mopidevi2023rmap,
  title={RMap: Millimeter-Wave Radar Mapping Through Volumetric Upsampling},
  author={Mopidevi, Ajay Narasimha and Harlow, Kyle and Heckman, Christoffer},
  journal={arXiv preprint arXiv:2310.13188},
  year={2023}
}
```
