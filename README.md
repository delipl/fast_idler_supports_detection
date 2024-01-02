# Usage

Running the algorithm and save output to the yaml file:
```bash
ros2 run objects_detection object_detection --ros-args -p general.filename:=second_part_with_etaps.yaml
```

Analyze the output yaml file:
```bash
ros2 run objects_detection clasification_analizer.py data/second_part_with_etaps.yaml
```
or
```bash
python objects_detection/scripts/clasification_analizer.py data/second_part_with_etaps.yaml
```

Make histograms and isometric view:
```bash
ros2 run objects_detection point_cloud_cli.py -i --his XY YZ -f objects_detection/test_data/tunneled_5.pcd
```
or
```bash
python objects_detection/scripts/point_cloud_cli.py -i --his XY YZ -f objects_detection/test_data/tunneled_5.pcd
```

Figures:
Figure 4.1:
```
python objects_detection/scripts/point_cloud_cli.py -i -f objects_detection/test_data/velodyne_raw_5.pcd
```
Figure 4.5:
```
python objects_detection/scripts/point_cloud_cli.py -i -f   ../../install/objects_detection/share/objects_detection/test_data/aligned_5.pcd
```

Figure 4.6:
```
python3  objects_detection/scripts/diff.py "Ground filter" ../../install/objects_detection/share/objects_detection/test_data/tunneled_5.pcd   ../../install/objects_detection/share/objects_detection/test_data/aligned_5.
```

Figure 4.7, 4.8, 4.10, 4.11:
```
python objects_detection/scripts/point_cloud_cli.py -i -his XY YZ -f ../../install/objects_detection/share/objects_detection/test_data/tunneled_5.pcd
```

Figure 4.9:
```
python3  objects_detection/scripts/diff.py "Density filter YZ" ../../install/objects_detection/share/objects_detection/test_data/density_filtered_yz_5.pcd   ../../install/objects_detection/share/objects_detection/test_data/tunneled_5.pcd
```

Figure 4.12:
```
python3  objects_detection/scripts/diff.py "Density filter XY" ../../install/objects_detection/share/objects_detection/test_data/density_filtered_xy_5.pcd   ../../install/objects_detection/share/objects_detection/test_data/tunneled_5.pcd
```

Figure 4.13:
```
python3  objects_detection/scripts/diff.py "Merged density filters" ../../install/objects_detection/share/objects_detection/test_data/merged_density_5.pcd   ../../install/objects_detection/share/objects_detection/test_data/tunneled_5.pcd
```

Figure 4.14:
```
python3  objects_detection/scripts/diff.py "Clustered supports" ../../install/objects_detection/share/objects_detection/test_data/merged_legs_5.pcd   ../../install/objects_detection/share/objects_detection/test_data/tunneled_5.pcd
```

Figure 4.15:
```

```