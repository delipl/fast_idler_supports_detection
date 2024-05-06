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

normalization_durations = np.array([])
density_segmentation_durations = np.array([])
supports_clusterization_durations = np.array([])
supports_classification_durations = np.array([])
estimation_durations = np.array([])
processing_durations = np.array([])

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
    normalization_durations = np.append(normalization_durations, float(frame["durations"]["normalization"]))
    density_segmentation_durations = np.append(density_segmentation_durations, float(frame["durations"]["density_segmentation"]))
    supports_clusterization_durations = np.append(supports_clusterization_durations, float(frame["durations"]["supports_clusterization"]))
    supports_classification_durations = np.append(supports_classification_durations, float(frame["durations"]["supports_classification"]))
    estimation_durations = np.append(estimation_durations, float(frame["durations"]["estimation"]))
    processing_durations = np.append(processing_durations, float(frame["durations"]["processing"]))


print(f"Loaded: {len(processing_durations)} scans")


# # Rysowanie wykresu długości callbacku od czasu
fig = plt.figure(f"Durations {sys.argv[1]}")
plt.rcParams.update({'font.size': 24})  # Ustawienie globalnie większego rozmiaru czcionki

ax = fig.add_subplot()


after_normalization_times = normalization_durations

after_density_segmentation_durations = (
    after_normalization_times + density_segmentation_durations
)
after_supports_clusterization_durations = (
    after_density_segmentation_durations + supports_clusterization_durations
)
after_supports_classification_durations = (
    after_supports_clusterization_durations + supports_classification_durations
)

width = 0.101

step_names = [
    "CA",
    "DS",
    "SCt",
    "SCs",
    "PE ",
    "Total",
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
    density_segmentation_durations,
    width=width,
    color="#FF4500",  # Zmieniony kolor na pomarańczowy
    label=step_names[1],
    bottom=after_normalization_times,
    align="edge",
)
plt.bar(
    bar_timestamps,
    supports_clusterization_durations,
    width=width,
    color="#8B4513",  # Zmieniony kolor na brązowy
    label=step_names[2],
    bottom=after_density_segmentation_durations,
    align="edge",
)
plt.bar(
    bar_timestamps,
    supports_classification_durations,
    width=width,
    color="#55A4C1",
    label=step_names[3],
    bottom=after_supports_clusterization_durations,
    align="edge",
)
plt.bar(
    bar_timestamps,
    estimation_durations,
    width=width,
    color="#9400D3",  # Zmieniony kolor na fioletowy
    label=step_names[4],
    bottom=after_supports_classification_durations,
    align="edge",
)


# Dodanie legendy
# plt.legend(bbox_to_anchor=(0.5, 1.05))
plt.legend()


ax.set_xlabel("time [s]")
ax.set_ylabel("processing time [s]")
ax.grid(True)

# Stamp check

fig, axs = plt.subplots(3)
axs[0].plot(secs)
axs[0].set_ylabel("discrete time [s]")

axs[1].set_title("Nanoseconds")
axs[1].plot(nanosecs)
axs[1].set_ylabel("time [ns]")

axs[2].plot(timestamps_ellips)
axs[2].set_xlabel("Message count")
axs[2].set_ylabel("time [s]")
plt.rcParams.update({'font.size': 24})  # Ustawienie globalnie większego rozmiaru czcionki




fig = plt.figure("Durations statistics")
table_data = [["process step", "min [ms]", "max [ms]", "mean [ms]", "std dev [ms]"]]
datas = [
    normalization_durations,
    density_segmentation_durations,
    supports_clusterization_durations,
    supports_classification_durations,
    estimation_durations,
    processing_durations,
]

for i in range(len(step_names)):
    data_array = np.array(datas[i])
    min_value = round(np.min(data_array)* 1e3 , 5)
    max_value = round(np.max(data_array)* 1e3, 5)
    mean_value = round(np.mean(data_array)* 1e3, 5)
    std_deviation = round(np.std(data_array)* 1e3, 5)
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

# Create boxplot for each parameter
plt.figure()


plt.rcParams.update({'font.size': 24})  # Ustawienie globalnie większego rozmiaru czcionki

plt.boxplot([normalization_durations, density_segmentation_durations, 
             supports_clusterization_durations, supports_classification_durations,
             estimation_durations, processing_durations], labels=step_names, showfliers=True)

plt.ylabel('Duration (seconds)')
plt.xticks(rotation=45)
plt.show()
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
