import numpy as np
from collections import defaultdict
from data_loader import DataLoader


class Feature:
    def __init__(self, box_center, box_size, mean_point, points_count, sec, nanosec):
        self.box_center = box_center
        self.box_size = box_size
        self.mean_point = mean_point
        self.points_count = points_count
        self.sec = sec
        self.nanosec = nanosec
        self.label = None

    def to_array(self):
        # Stack attributes vertically and then flatten
        x = np.vstack([self.box_center, self.box_size, self.mean_point]).flatten()
        return np.concatenate((x, np.array([self.points_count])))


def get_features_from_loader(loader: DataLoader):
    features = []
    while loader.next_object_feature():
        object = loader.get_object_feature()
        box_center = [
            object["box_center"]["x"],
            object["box_center"]["y"],
            object["box_center"]["z"],
        ]
        box_size = [
            object["box_size"]["x"],
            object["box_size"]["y"],
            object["box_size"]["z"],
        ]
        mean_point = [
            object["mean_point"]["x"],
            object["mean_point"]["y"],
            object["mean_point"]["z"],
        ]
        sec = object["timestamp"]["sec"]
        nanosec = object["timestamp"]["nanosec"]
        points_count = object["points_count"]
        feature = Feature(box_center, box_size, mean_point, points_count, sec, nanosec)
        features.append(feature)
    return features


def group_features_by_timestamp(tp_features, alg_features):
    timestamp_dict = defaultdict(defaultdict)
    print(f"Found {timestamp_dict.keys()} timestamps")

    for feature in tp_features:
        timestamp = str(feature.sec) + "_" + str(feature.nanosec)
        if len(timestamp_dict[timestamp]) == 0:
            timestamp_dict[timestamp] = defaultdict(list)

        timestamp_dict[timestamp]["tp"].append(feature)

    for feature in alg_features:
        timestamp = str(feature.sec) + "_" + str(feature.nanosec)
        if len(timestamp_dict[timestamp]) == 0:
            timestamp_dict[timestamp] = defaultdict(list)
        timestamp_dict[timestamp]["alg"].append(feature)
    return timestamp_dict


# method to check distace between bbox_center and point
def distance(bbox_center, point):
    return np.sqrt((bbox_center[0] - point[0]) ** 2 + (bbox_center[1] - point[1]) ** 2)


# Changed to check how far the the  poin t is
def is_point_in_bounding_box(point, box_size, bbox_center) -> bool:
    # print (f"Point: {point[0]}, {point[1]}, bbox_center: {bbox_center[0]}, {bbox_center[1]}, {distance(bbox_center, point)}")
    return abs(distance(bbox_center, point)) < 0.2
    # for i in range(2):
    #     # result = (point[i] < (bbox_center[i] - box_size[i]/2.0)) or (point[i] > (bbox_center[i] + box_size[i]/2.0))
    #     result = abs(distance(bbox_center, point)) > 0.2
    #     if result:
    #         return False
    # return True
