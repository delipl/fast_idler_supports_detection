import yaml
import numpy as np


def print_statistics(count):
    print("Min: ", np.min(count))
    print("Max: ", np.max(count))
    print("Mean: ", np.mean(count))
    print("Std dev: ", np.std(count))


# Sample YAML data
with open("./data/pass1_counts.yaml", "r", encoding="utf-8") as file:
    yaml_data = yaml.safe_load(file)

    # Calculate time differences in milliseconds
    original_points_count = []
    filter_further_than_5m_points_count = []
    filter_ground_points_count = []
    for i in range(0, len(yaml_data)):
        original_points_count.append(yaml_data[i]["original_points_count"])

        filter_further_than_5m_points_count.append(
            yaml_data[i]["filter_further_than_5m_points_count"]
        )
        filter_ground_points_count.append(yaml_data[i]["filter_ground_points_count"])

    print("Original points count: ", len(original_points_count))
    print_statistics(original_points_count)
    print("Filter further than 5m points count: ", len(filter_further_than_5m_points_count))
    print_statistics(filter_further_than_5m_points_count)
    print("Filter ground points count: ", len(filter_ground_points_count))
    print_statistics(filter_ground_points_count)
