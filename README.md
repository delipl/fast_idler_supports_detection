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