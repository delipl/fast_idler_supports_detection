#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import yaml
import sys
import numpy as np
from tabulate import tabulate

with open(sys.argv[1], "r") as file:
    data = yaml.safe_load(file)

# Przetwarzanie danych
timestamps_ellips = []
secs = []
nanosecs = []
durations_ellips = []
classified_legs_counts_elipoids = []
unclassified_legs_counts_elipoids = []
positions_ellips = []
radiuses_ellips = []
classes = []
classes_ids = []

normalization_durations = []
conveyor_clusterization_durations = []
conveyor_classification_durations = []
density_segmentation_durations = []
supports_clusterization_durations = []
supports_classification_durations = []
estimation_durations = []
processing_durations = []

point_original_sizes = []
point_5m_filter_sizes = []
point_ground_filter_sizes = []
point_roi_sizes = []

class_06_size = 0
class_07_size = 0
class_unknown_size = 0
# Dostęp do poszczególnych pól
for frame in data:
    sec = float(frame["timestamp"]["sec"])
    nanosec = float(frame["timestamp"]["nanosec"])

    secs.append(sec)
    nanosecs.append(nanosec)
    time_sec = (sec - secs[0]) + (nanosec-nanosecs[0])*1e-9
    timestamps_ellips.append(time_sec)
    classified_legs_counts_elipoids.append(0)
    unclassified_legs_counts_elipoids.append(0)
    normalization_durations.append(frame["durations"]["normalization"])
    conveyor_clusterization_durations.append(
        float(frame["durations"]["conveyor_clusterization"])
    )
    conveyor_classification_durations.append(
        float(frame["durations"]["conveyor_classification"])
    )
    density_segmentation_durations.append(
        float(frame["durations"]["density_segmentation"])
    )
    supports_clusterization_durations.append(
        float(frame["durations"]["supports_clusterization"])
    )
    supports_classification_durations.append(
        float(frame["durations"]["supports_classification"])
    )
    estimation_durations.append(float(frame["durations"]["estimation"]))
    processing_durations.append(float(frame["durations"]["processing"]))

    # point_original_sizes.append(frame["filters_point_sizes"]["original"])
    # point_5m_filter_sizes.append(frame["filters_point_sizes"]["5m_filter"])
    # point_ground_filter_sizes.append(frame["filters_point_sizes"]["ground_filter"])
    # point_roi_sizes.append(frame["filters_point_sizes"]["roi"])

    # try:
    #     for ellipsoid in frame["detected_ellipsoids"]:
    #         positions_ellips.append(ellipsoid["position"])
    #         radiuses_ellips.append(ellipsoid["major_axes"])
    #         classes.append(ellipsoid["class"])
    #         if ellipsoid["class"] == "0.6m_height_support":
    #             classes_ids.append("green")
    #             class_06_size += 1
    #             classified_legs_counts_elipoids[-1] += 1
    #         elif ellipsoid["class"] == "0.7m_height_support":
    #             classes_ids.append("blue")
    #             class_07_size+=1
    #             classified_legs_counts_elipoids[-1] += 1
    #         else:
    #             class_unknown_size+=1
    #             classes_ids.append("gray")
    #             unclassified_legs_counts_elipoids[-1] += 1

    # except KeyError:
    #     continue
print(f"Loaded: {len(processing_durations)} scans")
# Tworzenie subplotów
# fig = plt.figure(f"Detected and classified supports {sys.argv[1]}")
# widths = [2.5, 1.8]
# heights = [1.5, 1.5, 1.0]
# gs = fig.add_gridspec(ncols=2, nrows=3, width_ratios=widths, height_ratios=heights)

# gs = fig.add_gridspec(3, 3)
# fig, axs = plt.subplots(2, 2, figsize=(15, 10))
# ax = fig.add_subplot(projection="3d")
# Rysowanie wykresu 3D
# ax.scatter(
#     [pos["x"] for pos in positions_ellips],
#     [pos["y"] for pos in positions_ellips],
#     [pos["z"] for pos in positions_ellips],
#     marker=".",
#     alpha=0.2,
#     c=classes_ids,
# )

# ax.set_xlabel("X", fontsize=8)
# ax.set_ylabel("Y", fontsize=8)
# ax.set_zlabel("Z", fontsize=8)

# ax.xaxis.pane.fill = False
# ax.yaxis.pane.fill = False
# ax.zaxis.pane.fill = False
# ax.set_xlim([0.0, 5.0])
# ax.set_ylim([-2, 2])
# ax.set_zlim([0.0, 1.2])


# red_patch = mpatches.Patch(color='gray', label=f'unclassified [{class_unknown_size}]')
# green_patch = mpatches.Patch(color='green', label=f'classified 0.6m support [{class_06_size}]')
# yellow_patch = mpatches.Patch(color='blue', label=f'classified 0.7m support [{class_07_size}]')
# ax.legend(handles=[red_patch, green_patch, yellow_patch])
# ax.set_title("Detected and classified supports")


# plt.legend()

# fig = plt.figure(f"Classified supports {sys.argv[1]}")
# # ax.view_init(azim=180, elev=50)
# # # ax.set(title='Pozycje elipsoid', xlabel='X [m]', ylabel='Y [m]')
# # ax.grid(True)


# # # Rysowanie wykresu liczby nóg
# ax = fig.add_subplot()
# ax.plot(
#     timestamps_ellips,
#     classified_legs_counts_elipoids,
#     marker=".",
#     linestyle="",
#     color="g",
# )
# ax.plot(
#     timestamps_ellips,
#     unclassified_legs_counts_elipoids,
#     marker=".",
#     linestyle="",
#     color="r",
# )
# ax.set_title("Classified supports")
# ax.set_xlabel("time [s]")
# ax.set_ylabel("number of classified supports")

# ax.grid(True)


# # Rysowanie wykresu długości callbacku od czasu
fig = plt.figure(f"Durations {sys.argv[1]}")
ax = fig.add_subplot()
normalization_durations = np.array(normalization_durations)
conveyor_clusterization_durations = np.array(conveyor_clusterization_durations)
conveyor_classification_durations = np.array(conveyor_classification_durations)
density_segmentation_durations = np.array(density_segmentation_durations)
supports_clusterization_durations = np.array(supports_clusterization_durations)
supports_classification_durations = np.array(supports_classification_durations)
estimation_durations = np.array(estimation_durations)
processing_durations = np.array(processing_durations)


after_normalization_times = normalization_durations
after_conveyor_clusterization_durations = (
    after_normalization_times + conveyor_clusterization_durations
)
after_conveyor_classification_durations = (
    after_conveyor_clusterization_durations + conveyor_classification_durations
)
after_density_segmentation_durations = (
    after_conveyor_classification_durations + density_segmentation_durations
)
after_supports_clusterization_durations = (
    after_conveyor_classification_durations + supports_clusterization_durations
)
after_supports_classification_durations = (
    after_supports_clusterization_durations + supports_classification_durations
)

width = 0.101

step_names = [
    "Cropping and alignment",
    "Conveyor clusterization",
    "Conveyor classification",
    "Density segmentation",
    "Supports clusterization",
    "Supports classification",
    "Position estimation",
    "Whole process",
]

bar_timestamps = np.array(timestamps_ellips)
# bar_timestamps = np.linspace(0.0, 1.0*len(timestamps_ellips)/10, num=len(timestamps_ellips))
plt.bar(
    bar_timestamps,
    normalization_durations,
    width=width,
    color="#FFD700",  # Zmieniony kolor na złoty
    label=step_names[0],
    align="edge",
)
plt.bar(
    bar_timestamps,
    conveyor_clusterization_durations,
    width=width,
    color="#FFB6C1",
    label=step_names[1],
    bottom=after_normalization_times,
    align="edge",
)
plt.bar(
    bar_timestamps,
    conveyor_classification_durations,
    width=width,
    color="#00B6C1",
    label=step_names[2],
    bottom=after_conveyor_clusterization_durations,
    align="edge",
)
plt.bar(
    bar_timestamps,
    density_segmentation_durations,
    width=width,
    color="#FF4500",  # Zmieniony kolor na pomarańczowy
    label=step_names[3],
    bottom=after_conveyor_classification_durations,
    align="edge",
)
plt.bar(
    bar_timestamps,
    supports_clusterization_durations,
    width=width,
    color="#8B4513",  # Zmieniony kolor na brązowy
    label=step_names[4],
    bottom=after_density_segmentation_durations,
    align="edge",
)
plt.bar(
    bar_timestamps,
    supports_classification_durations,
    width=width,
    color="#55A4C1",
    label=step_names[5],
    bottom=after_supports_clusterization_durations,
    align="edge",
)
plt.bar(
    bar_timestamps,
    estimation_durations,
    width=width,
    color="#9400D3",  # Zmieniony kolor na fioletowy
    label=step_names[6],
    bottom=after_supports_classification_durations,
    align="edge",
)


# Dodanie legendy
# plt.legend(bbox_to_anchor=(0.5, 1.05))
plt.legend()
plt.legend()


ax.set_title("Algorithm's processing time")
ax.set_xlabel("time [s]")
ax.set_ylabel("processing time [s]")
ax.grid(True)

# Stamp check

fig, axs = plt.subplots(3)
axs[0].set_title("Seconds")
axs[0].plot(secs)
axs[0].set_ylabel("discrete time [s]")

axs[1].set_title("Nanoseconds")
axs[1].plot(nanosecs)
axs[1].set_ylabel("time [ns]")

axs[2].set_title("Timestamp")
axs[2].plot(timestamps_ellips)
axs[2].set_xlabel("Message count")
axs[2].set_ylabel("time [s]")

# # Rysowanie histogramu liczby nóg
# fig = plt.figure()
# ax = fig.add_subplot()
# ax.hist(
#     classified_legs_counts_elipoids,
#     bins=np.arange(0, max(classified_legs_counts_elipoids) + 2, 1),
#     align="left",
#     color="green",
# )
# ax.set_title("Number of classified supports in measurements")
# ax.set_xlabel("number of classified supports")
# ax.set_ylabel("number of measurements")
# ax.grid(True)

# # Rysowanie histogramu liczby nóg
# ax = fig.add_subplot(gs[2, 1])
# ax.hist(
#     unclassified_legs_counts_elipoids,
#     bins=np.arange(0, max(unclassified_legs_counts_elipoids) + 2, 1),
#     align="left",
#     color="red",
# )
# ax.set_title("Number of unclassified supports in measurements")
# ax.set_xlabel("number of unclassified supports")
# ax.set_ylabel("number of measurements")
# ax.grid(True)
# plt.tight_layout()

# fig = plt.figure("Statistics")
# ax = fig.add_subplot()
# ax.axis("off")
# ax.axis("tight")
# plt.title(f"Time tables {sys.argv[1]}")
# gs = fig.add_gridspec(ncols=1, nrows=2)
# ax = fig.add_subplot(gs[0, 0])

# fig.patch.set_visible(False)
# ax.axis("off")
# ax.axis("tight")
fig = plt.figure("Durations statistics")
table_data = [["process step", "min [ms]", "max [ms]", "mean [ms]", "std dev [ms]"]]
datas = [
    normalization_durations,
    conveyor_clusterization_durations,
    conveyor_classification_durations,
    density_segmentation_durations,
    supports_clusterization_durations,
    supports_classification_durations,
    estimation_durations,
    processing_durations,
]

for i in range(len(step_names)):
    min_value = round(np.min(datas[i]*1e3), 5)
    max_value = round(np.max(datas[i]*1e3), 5)
    mean_value = round(np.mean(datas[i]*1e3), 5)
    std_deviation = round(np.std(datas[i]*1e3), 5)
    table_data.append([step_names[i], min_value, max_value, mean_value, std_deviation])

table = plt.table(
    cellText=table_data, loc="upper right", cellLoc="center", colLabels=None
)

table.auto_set_font_size(False)
table.set_fontsize(10)
latex_table = tabulate(table_data, headers=["Statystyka", "Wartość"], tablefmt="latex")

# Zapisywanie tabeli do pliku
with open(f"{sys.argv[1]}_time_table.tex", "w") as file:
    file.write(latex_table)
# table.scale(1, 1.5)
# # Dostosowanie odstępów między subplotami
# plt.tight_layout()
# fig = plt.figure("Statistics")
# ax = fig.add_subplot(gs[1, 0])
# plt.title(f"Counts tables {sys.argv[1]}")

# ax.axis("off")
# ax.axis("tight")
# table_data = [["filtration step", "min", "max", "mean", "std dev"]]
# step_names = ["Original", "5m Filter", "Ground Filter", "ROI"]
# datas = [
#     point_original_sizes,
#     point_5m_filter_sizes,
#     point_ground_filter_sizes,
#     point_roi_sizes,
# ]
# for i in range(len(step_names)):
#     min_value = np.min(datas[i])
#     max_value = np.max(datas[i])
#     mean_value = int(np.mean(datas[i]))
#     std_deviation = int(np.std(datas[i]))
#     table_data.append([step_names[i], min_value, max_value, mean_value, std_deviation])

# table = ax.table(
#     cellText=table_data, loc="upper right", cellLoc="center", colLabels=None
# )

# table.auto_set_font_size(False)
# table.set_fontsize(10)
# latex_table = tabulate(table_data, headers=["Statystyka", "Wartość"], tablefmt="latex")

# # Zapisywanie tabeli do pliku
# with open(f"{sys.argv[1]}_counts_table.tex", "w") as file:
#     file.write(latex_table)
# table.scale(1, 1.5)
# # Dostosowanie odstępów między subplotami
# plt.tight_layout()

# # Wyświetlenie wykresów
plt.show()
