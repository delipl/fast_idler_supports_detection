import os
import yaml
from data_splitter import validate_data_name, validate_path


class DataLoader:
    def __init__(self, data_path: str, data_file_name: str) -> None:
        self.data_path = validate_path(data_path)
        self.data_file_name = validate_data_name(data_file_name)
        self.data_split_path = self.data_path + data_file_name + "_split/"

        if not os.path.exists(self.data_split_path):
            print(f"There is not split data directory. Path {self.data_split_path}")

        feature_dirs = []
        all_dirs = []
        self.durations_paths = []
        for root, dirs, files in os.walk(self.data_split_path):
            if "durations.yaml" in files:
                self.durations_paths.append(os.path.join(root, "durations.yaml"))

            for timestamp_dir in dirs:
                if timestamp_dir.endswith("features"):
                    feature_dirs.append(os.path.join(root, timestamp_dir))
                if timestamp_dir.endswith("all"):
                    all_dirs.append(os.path.join(root, timestamp_dir))

        self.feature_files_paths = self.read_dirs(feature_dirs)
        self.all_files_paths = self.read_dirs(all_dirs)

        self.actual_feature_file = None
        self.actual_all_file = None
        self.actual_feature_node = None
        self.actual_all_node = None

    def read_dirs(self, dirs: os.PathLike) -> list:
        files_paths = []
        for feature_dir in dirs:
            for root, _, files in os.walk(feature_dir):
                for file in files:
                    files_paths.append(os.path.join(root, file))

        return files_paths

    def get_object_feature(self):
        if self.actual_feature_file is None and len(self.feature_files_paths) > 0:
            self.actual_feature_file = self.feature_files_paths[0]
            self.feature_files_paths = self.feature_files_paths[1:]

        with open(self.actual_feature_file, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)

    def next_object_feature(self):
        if len(self.feature_files_paths) > 0:
            self.actual_feature_file = self.feature_files_paths[0]
            self.feature_files_paths = self.feature_files_paths[1:]
            return True
        return False

    def get_object_all(self):
        if self.actual_all_file is None and len(self.all_files_paths) > 0:
            self.actual_all_file = self.all_files_paths[0]
            self.all_files_paths = self.all_files_paths[1:]
            print(self.all_files_paths)

        with open(self.actual_all_file, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)

    def next_object_all(self) -> bool:
        if len(self.all_files_paths) > 0:
            self.actual_all_file = self.all_files_paths[0]
            self.all_files_paths = self.all_files_paths[1:]
            return True
        return False

    def accumulate_durations(self) -> yaml.Node:
        accumulated_node = {}
        for durations_path in self.durations_paths:
            with open(durations_path, "r", encoding="utf-8") as file:
                node = yaml.safe_load(file)
                for key in node.keys():
                    if key in accumulated_node:
                        accumulated_node[key] += node[key]
                    else:
                        accumulated_node[key] = node[key]
        return accumulated_node
