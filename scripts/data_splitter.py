import yaml
import os
import csv
from copy import copy
import shutil


def validate_path(path: str) -> str:
    if path[-1] != "/":
        path += "/"

    return path


def validate_data_name(name: str) -> str:
    if name[-5:] == ".yaml":
        name = name[:-5]

    return name


class DataSplitter:
    def __init__(self, data_file_path: str, data_file_name: str) -> None:
        self.data_file_path = validate_path(data_file_path)
        self.data_file_name = validate_data_name(data_file_name)

    def try_to_mkdir(self, path: os.PathLike) -> bool:
        if not os.path.exists(path):
            os.mkdir(path)
            return True
        return False

    def use_timestamp_dir(self, node: yaml.Node, split_data_path: os.PathLike) -> os.PathLike:
        timestamp = node["timestamp"]
        timestamp_folder_name = str(timestamp["sec"]) + "_" + str(timestamp["nanosec"])
        timestamp_path = os.path.join(split_data_path, timestamp_folder_name)
        self.try_to_mkdir(timestamp_path)

        return timestamp_path

    def save_object_to_file(self, obj: yaml.Node, timestamp_path: os.PathLike, name: str) -> None:
        object_all_path = os.path.join(
            timestamp_path,
            "all",
        )
        object_features_path = os.path.join(timestamp_path, "features")
        self.try_to_mkdir(object_all_path)
        self.try_to_mkdir(object_features_path)
        object_all_path = os.path.join(object_all_path, name + ".yaml")
        object_features_path = os.path.join(object_features_path, name + ".yaml")

        with open(object_all_path, "w", encoding="utf-8") as file:
            yaml.dump(obj, file, default_flow_style=False)

        try:
            obj["points_count"] = len(obj["points"])
            del obj["points"]
        except KeyError:
            print(f"Could not find points for this object:\n{obj}")
            os.remove(object_all_path)
            return

        with open(object_features_path, "w", encoding="utf-8") as file:
            yaml.dump(obj, file, default_flow_style=False)

    def read_yaml(self, data_file_path: str, data_file_name: str) -> yaml.Node:
        with open(data_file_path + data_file_name + ".yaml", "r", encoding="utf-8") as file:
            return yaml.safe_load(file)

    def save_durations(self, timestamp_path: os.PathLike, node: yaml.Node):
        durations_path = os.path.join(timestamp_path, "durations.yaml")
        with open(durations_path, "w", encoding="utf-8") as file:
            yaml.dump(node, file, default_flow_style=False)

    def split_data_from_fast_idler_supports_detection_node(
        self, data_file_path: str, data_file_name: str
    ):
        data = self.read_yaml(data_file_path, data_file_name)

        split_data_path = os.path.join(data_file_path, data_file_name + "_split")
        self.try_to_mkdir(split_data_path)

        for data_at_timestamp in data:
            timestamp_path = self.use_timestamp_dir(copy(data_at_timestamp), split_data_path)
            timestamp_sec = data_at_timestamp["timestamp"]["sec"]
            timestamp_nanosec = data_at_timestamp["timestamp"]["nanosec"]

            durations = data_at_timestamp["durations"]
            self.save_durations(timestamp_path, durations)

            index = 0
            print(timestamp_path)

            try:
                objects = data_at_timestamp["objects"]
            except KeyError:
                print(f"No objects found at timestamp:\n{data_at_timestamp}")
                continue

            for obj in objects:
                obj["timestamp"] = {"sec": timestamp_sec, "nanosec": timestamp_nanosec}

                self.save_object_to_file(obj, timestamp_path, "object_" + str(index))
                index += 1

    def split_node_data(self):
        self.split_data_from_fast_idler_supports_detection_node(
            self.data_file_path, self.data_file_name
        )

    def split_data_from_rviz_selection_node(
        self, data_file_path: str, data_file_name: str
    ) -> None:
        data = self.read_yaml(data_file_path, data_file_name)

        split_data_path = os.path.join(data_file_path, data_file_name + "_split")
        if not self.try_to_mkdir(split_data_path):
            print(f"Split folder exists {split_data_path}. Skipping...")
            return

        for selection in data:
            timestamp_path = self.use_timestamp_dir(selection, split_data_path)

            index = None
            try:
                all_files_path = os.path.join(timestamp_path, "all")
                list_of_files = os.listdir(all_files_path)
                index = len(list_of_files)
            except FileNotFoundError:
                index = 0

            self.save_object_to_file(selection, timestamp_path, "object_" + str(index))

    def split_rviz_selection_data(self):
        self.split_data_from_rviz_selection_node(self.data_file_path, self.data_file_name)

    def merge_features_into_csv(self) -> None:
        split_data_path = os.path.join(self.data_file_path, self.data_file_name + "_split")
        list_of_timestamp_dirs = os.listdir(split_data_path)

        csv_filename = os.path.join(split_data_path, "features.csv")
        if os.path.exists(csv_filename):
            print(f"CSV with features exists {csv_filename}, Skipping...")
            return

        with open(csv_filename, mode="a", newline="\n", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "timestamp_sec",
                    "timestamp_nanosec",
                    "frame_id",
                    "mean_point_x",
                    "mean_point_y",
                    "mean_point_z",
                    "box_size_x",
                    "box_size_y",
                    "box_size_z",
                    "points_count",
                ]
            )

        for timestamp_dir in list_of_timestamp_dirs:
            features_path = os.path.join(split_data_path, timestamp_dir, "features")
            if not os.path.isfile(timestamp_dir):
                print(f"Looking for files at {features_path}")

                try:
                    objects_files = os.listdir(features_path)
                except FileNotFoundError:
                    print(f"No files found at {features_path}")
                    continue

                for object_file in objects_files:
                    features_path = validate_path(features_path)
                    object_file = validate_data_name(object_file)

                    features = self.read_yaml(features_path, object_file)
                    with open(csv_filename, mode="a", newline="\n", encoding="utf-8") as file:
                        writer = csv.writer(file)
                        csv_data = [
                            features["timestamp"]["sec"],
                            features["timestamp"]["nanosec"],
                            features["frame_id"],
                            features["mean_point"]["x"],
                            features["mean_point"]["y"],
                            features["mean_point"]["z"],
                            features["box_size"]["x"],
                            features["box_size"]["y"],
                            features["box_size"]["z"],
                            features["points_count"],
                        ]
                        writer.writerow(csv_data)

    def clear(self) -> None:
        split_data_path = os.path.join(self.data_file_path, self.data_file_name + "_split")
        shutil.rmtree(split_data_path)
