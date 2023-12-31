#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import yaml
import sys
import numpy as np

with open(sys.argv[1], "r") as file:
    data = yaml.safe_load(file)

# Przetwarzanie danych
timestamps_ellips = []
durations_ellips = []
classified_legs_counts_elipoids = []
unclassified_legs_counts_elipoids = []
positions_ellips = []
radiuses_ellips = []
classes = []
classes_ids = []

normalization_durations = []
density_segmentation_durations = []
clusterization_durations = []
classification_durations = []
estimation_durations = []
processing_durations = []

class_06_size=0
class_07_size=0
class_unknown_size=0
# Dostęp do poszczególnych pól
for frame in data:
    timestamps_ellips.append(frame["time"] - data[0]["time"])
    classified_legs_counts_elipoids.append(0)
    unclassified_legs_counts_elipoids.append(0)
    normalization_durations.append(frame["normalization_duration"])
    density_segmentation_durations.append(frame["density_segmentation_duration"])
    clusterization_durations.append(frame["clusterization_duration"])
    classification_durations.append(frame["classification_duration"])
    estimation_durations.append(frame["estimation_duration"])
    processing_durations.append(frame["processing_duration"])

    try:
        for ellipsoid in frame["detected_ellipsoids"]:
            positions_ellips.append(ellipsoid["position"])
            radiuses_ellips.append(ellipsoid["major_axes"])
            classes.append(ellipsoid["class"])
            if ellipsoid["class"] == "0.6m_height_support":
                classes_ids.append("green")
                class_06_size += 1
                classified_legs_counts_elipoids[-1] += 1
            elif ellipsoid["class"] == "0.7m_height_support":
                classes_ids.append("blue")
                class_07_size+=1
                classified_legs_counts_elipoids[-1] += 1
            else:
                class_unknown_size+=1
                classes_ids.append("gray")
                unclassified_legs_counts_elipoids[-1] += 1

    except KeyError:
        continue

# Tworzenie subplotów
fig = plt.figure(sys.argv[1], figsize=plt.figaspect(0.5))
gs = fig.add_gridspec(3, 2)
# fig, axs = plt.subplots(2, 2, figsize=(15, 10))
ax = fig.add_subplot(gs[:2, 0], projection="3d")
# Rysowanie wykresu 3D
ax.scatter(
    [pos["x"] for pos in positions_ellips],
    [pos["y"] for pos in positions_ellips],
    [pos["z"] for pos in positions_ellips],
    marker=".",
    alpha=0.2,
    c=classes_ids,
)

ax.set_xlabel("X", fontsize=8)
ax.set_ylabel("Y", fontsize=8)
ax.set_zlabel("Z", fontsize=8)

ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False
ax.set_xlim([0.0, 5.0])
ax.set_ylim([-2, 2])
ax.set_zlim([0.0, 1.2])


red_patch = mpatches.Patch(color='gray', label=f'unclassified [{class_unknown_size}]')
green_patch = mpatches.Patch(color='green', label=f'classified 0.6m support [{class_06_size}]')
yellow_patch = mpatches.Patch(color='blue', label=f'classified 0.7m support [{class_07_size}]')
ax.legend(handles=[red_patch, green_patch, yellow_patch])


# plt.legend()


ax.view_init(azim=-155, elev=20)
ax.set_title("Detected and classified supports")
# ax.set(title='Pozycje elipsoid', xlabel='X [m]', ylabel='Y [m]')
ax.grid(True)


# # Rysowanie wykresu liczby nóg
ax = fig.add_subplot(gs[0, 1])
ax.plot(
    timestamps_ellips,
    classified_legs_counts_elipoids,
    marker=".",
    linestyle="",
    color="g",
)
ax.plot(
    timestamps_ellips,
    unclassified_legs_counts_elipoids,
    marker=".",
    linestyle="",
    color="r",
)
ax.set_title("Classified supports")
ax.set_xlabel("time [s]")
ax.set_ylabel("number of classified supports")

ax.grid(True)


# # Rysowanie wykresu długości callbacku od czasu
ax = fig.add_subplot(gs[1, 1])
after_normalization_times = normalization_durations
after_density_segmentation_times = np.array(normalization_durations) + np.array(
    density_segmentation_durations
)
after_clusterization_times = after_density_segmentation_times + np.array(clusterization_durations)
after_estimation_times = after_clusterization_times + np.array(estimation_durations)
after_classification_times = after_estimation_times + np.array(classification_durations)
width = 0.101
normalization_durations = np.array(normalization_durations)
density_segmentation_durations = np.array(density_segmentation_durations)
clusterization_durations = np.array(clusterization_durations)
classification_durations = np.array(classification_durations)
estimation_durations = np.array(estimation_durations)
processing_durations = np.array(processing_durations)
bar_timestamps = np.array(timestamps_ellips) + width / 2

plt.bar(
    bar_timestamps,
    normalization_durations,
    width=width,
    color="#EAE0C8",
    label="normalization",
    align="edge",
)
plt.bar(
    bar_timestamps,
    density_segmentation_durations,
    width=width,
    color="#FFB6C1",
    label="density segmentation",
    bottom=after_normalization_times,
    align="edge",
)
plt.bar(
    bar_timestamps,
    clusterization_durations,
    width=width,
    color="#ADD8E6",
    label="clusterization",
    bottom=after_density_segmentation_times,
    align="edge",
)
plt.bar(
    bar_timestamps,
    estimation_durations,
    width=width,
    color="#D8BFD8",
    label="estimation",
    bottom=after_clusterization_times,
    align="edge",
)
plt.bar(
    bar_timestamps,
    classification_durations,
    width=width,
    color="#DC143C",
    label="classification",
    bottom=after_estimation_times,
    align="edge",
)

# Dodanie legendy
plt.legend()


ax.set_title("Algorithm's processing time")
ax.set_xlabel("time [s]")
ax.set_ylabel("processing time [s]")
ax.grid(True)

# # Rysowanie histogramu liczby nóg
ax = fig.add_subplot(gs[2, 0])
ax.hist(
    classified_legs_counts_elipoids,
    bins=np.arange(0, max(classified_legs_counts_elipoids) + 2, 1),
    align="left",
    color="green",
)
ax.set_title("Number of classified supports in measurements")
ax.set_xlabel("number of classified supports")
ax.set_ylabel("number of measurements")
ax.grid(True)

# # Rysowanie histogramu liczby nóg
ax = fig.add_subplot(gs[2, 1])
ax.hist(
    unclassified_legs_counts_elipoids,
    bins=np.arange(0, max(unclassified_legs_counts_elipoids) + 2, 1),
    align="left",
    color="red",
)
ax.set_title("Number of unclassified supports in measurements")
ax.set_xlabel("number of unclassified supports")
ax.set_ylabel("number of measurements")
ax.grid(True)

# # Dostosowanie odstępów między subplotami
plt.tight_layout()

# # Wyświetlenie wykresów
plt.show()
