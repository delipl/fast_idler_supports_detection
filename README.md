# fast_idler_supports_detection

This package provides tools for fast idler supports detection using ROS 2 with PCL and density histograms.

## Installation

Clone the repository and its submodules:

```bash
git clone git@git.kcir.pwr.edu.pl:jdelicat/idlers_detection.git --recursive
```

Install dependencies using `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon  build --symlink-install
source install/setup.bash
```

## Running the Algorithm

Run the detection algorithm and save the output to a YAML file:

```bash
ros2 launch fast_idler_supports_detection fast_idler_supports_detection.launch.py
```


## Figures

Running the algorithm and save output to the yaml file:
```bash
ros2 run fast_idler_supports_detection fast_idler_supports_detection --ros-args -p general.filename:=second_part_with_etaps.yaml
```

Analyze the output yaml file:
```bash
ros2 run fast_idler_supports_detection clasification_analizer.py data/second_part_with_etaps.yaml
```
or
```bash
python fast_idler_supports_detection/scripts/clasification_analizer.py data/second_part_with_etaps.yaml
```

Make histograms and isometric view:
```bash
ros2 run fast_idler_supports_detection point_cloud_cli.py -i --his XY YZ -f fast_idler_supports_detection/test_data/tunneled_5.pcd
```
or
```bash
python fast_idler_supports_detection/scripts/point_cloud_cli.py -i --his XY YZ -f fast_idler_supports_detection/test_data/tunneled_5.pcd
```

Figures:
Figure 4.1:
```
python fast_idler_supports_detection/scripts/point_cloud_cli.py -i -f fast_idler_supports_detection/test_data/velodyne_raw_5.pcd
```
Figure 4.5:
```
python fast_idler_supports_detection/scripts/point_cloud_cli.py -i -f   ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/aligned_5.pcd
```

Figure 4.6:
```
python3  fast_idler_supports_detection/scripts/diff.py "Ground filter" ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/tunneled_5.pcd   ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/aligned_5.
```

Figure 4.7, 4.8, 4.10, 4.11:
```
python fast_idler_supports_detection/scripts/point_cloud_cli.py -i -his XY YZ -f ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/tunneled_5.pcd
```

Figure 4.9:
```
python3  fast_idler_supports_detection/scripts/diff.py "Density filter YZ" ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/density_filtered_yz_5.pcd   ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/tunneled_5.pcd
```

Figure 4.12:
```
python3  fast_idler_supports_detection/scripts/diff.py "Density filter XY" ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/density_filtered_xy_5.pcd   ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/tunneled_5.pcd
```

Figure 4.13:
```
python3  fast_idler_supports_detection/scripts/diff.py "Merged density filters" ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/merged_density_5.pcd   ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/tunneled_5.pcd
```

Figure 4.14:
```
python3  fast_idler_supports_detection/scripts/diff.py "Clustered supports" ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/merged_legs_5.pcd   ../../install/fast_idler_supports_detection/share/fast_idler_supports_detection/test_data/tunneled_5.pcd
```

Figure 4.15:
```

```

Table processing time:
```
ros2 run fast_idler_supports_detection clasification_analizer.py data/first_part_with_etaps.yaml
```
