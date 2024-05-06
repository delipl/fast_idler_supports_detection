import matplotlib.pyplot as plt
import numpy as np


def show_2D_YZ_histogram(cloud, resolution=0.1):
    histogram_image = []
    y = cloud[:, 1]
    z = cloud[:, 2]
    # print(f"min x {min(x)}, max x {max(x) max y }")
    width = 2 * max(y)
    height = max(z)
    image_width = int(width / resolution)
    image_height = int(height / resolution)
    max_sector = 0

    for _ in range(image_height):
        column = [0] * image_width
        histogram_image.append(column)

    for point in cloud:
        image_width_pos = int((width / 2 + point[1]) / resolution)
        image_height_pos = int(image_height) - int(point[2] / resolution) - 1
        if image_height_pos < 0 or image_width_pos < 0:
            continue
        if image_width_pos >= image_width or image_height_pos >= image_height:
            continue
        histogram_image[image_height_pos][image_width_pos] += 1
        max_sector = max([histogram_image[image_height_pos][image_width_pos], max_sector])
    plt.rcParams.update({"font.size": 22})  # Ustawienie globalnie większego rozmiaru czcionki

    plt.figure("2D histogram YZ")
    plt.xlabel("Y")
    plt.ylabel("Z")
    extent_values = [min(y), max(y), min(z), max(z)]
    print(f"min(y): {min(y)}, max(y): {max(y)}, min(z): {min(z)}, max(z): {max(z)}")

    img = plt.imshow(histogram_image, extent=extent_values)
    img.set_clim(0, max_sector)
    plt.colorbar()
    plt.rcParams.update({"font.size": 22})  # Ustawienie globalnie większego rozmiaru czcionki

    plt.figure("Histogram YZ")
    plt.xlabel("number of points in a cell", fontsize=12)
    plt.ylabel("number of cells", fontsize=12)
    plt.grid()
    densities = []
    for column in histogram_image:
        column = [x for x in column if x != 0]
        densities += column
    densities = np.array(densities)

    plt.hist(densities, bins=np.arange(min(densities), max(densities) + 1, 1), align="left")
    plt.axvline(x=30, color="r", linestyle="-")
    plt.xticks(np.arange(min(densities), max(densities) + 1, 11))

    return histogram_image


def show_2D_XY_histogram(cloud, resolution=0.1):
    histogram_image = []
    x = cloud[:, 0]
    y = cloud[:, 1]
    width = 2 * max(y)
    height = max(x)
    max_sector = 0

    image_width = int(width / resolution)
    image_height = int(height / resolution)

    for _ in range(image_height):
        column = [0] * image_width
        histogram_image.append(column)

    for point in cloud:
        image_width_pos = int(image_width / 2) - int(point[1] / resolution)
        image_height_pos = int(image_height) - int(point[0] / resolution) - 1
        if image_height_pos < 0 or image_width_pos < 0:
            continue
        if image_width_pos >= image_width or image_height_pos >= image_height:
            continue
        histogram_image[image_height_pos][image_width_pos] += 1
        max_sector = max([histogram_image[image_height_pos][image_width_pos], max_sector])
    plt.rcParams.update({"font.size": 22})  # Ustawienie globalnie większego rozmiaru czcionki

    plt.figure("2D histogram XY")
    plt.xlabel("Y")
    plt.ylabel("X")
    extent_values = [min(y), max(y), min(x), max(x)]
    img = plt.imshow(histogram_image, extent=extent_values)
    img.set_clim(0, max_sector)
    plt.colorbar()
    plt.rcParams.update({"font.size": 22})  # Ustawienie globalnie większego rozmiaru czcionki

    plt.figure("Histogram XY")
    plt.xlabel("number of points in a cell")
    plt.ylabel("number of cells")
    plt.grid()

    densities = []
    for column in histogram_image:
        column = [x for x in column if x != 0]
        densities += column
    densities = np.array(densities)

    plt.hist(densities, bins=np.arange(min(densities), max(densities) + 1, 1), align="left")
    plt.xticks(np.arange(min(densities), max(densities) + 1, 11))

    return histogram_image
