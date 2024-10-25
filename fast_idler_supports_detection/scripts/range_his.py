import matplotlib.pyplot as plt
import numpy as np
import math
import copy


def cart_to_sphere(x, y, z):
    r = math.sqrt(x * x + y * y + z * z)
    a = 0.0
    b = 0.0
    if x != 0:
        a = math.atan(y / x)
    if r != 0:
        b = math.asin(z / r)  # bo jest obrócony

    return np.array([r, a, b])


def rotate_and_translate(points, angle, d):
    rotation_matrix = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )

    rotated_points = np.dot(points, rotation_matrix.T)

    translation_vector = np.array([0, 0, d])
    translated_points = rotated_points + translation_vector

    return translated_points


def show_2D_XY_histogram(resolution=0.1, width=2.0, height=2.0):
    histogram_image_min = []
    histogram_image_max = []
    histogram_image_height = []

    image_width = int(height / resolution)
    image_height = int(width / resolution)
    print(f"{image_width}, {image_height}")
    for i in range(image_width):
        column = [0] * image_height
        histogram_image_min.append(copy.copy(column))
        histogram_image_max.append(copy.copy(column))
        histogram_image_height.append(copy.copy(column))

    for i in range(image_width):
        for j in range(image_height):
            # z_offset = -1.34
            model_height = 0.6
            fov = 15.0
            intensity = 0
            offset = math.degrees(0.50)
            point_min = np.array([i * resolution, j * resolution - width / 2.0, 0])
            point_max = np.array([i * resolution, j * resolution - width / 2.0, model_height])

            origin_points = rotate_and_translate([point_min, point_max], 0.5, -1.34)

            sphere_min_config = cart_to_sphere(
                origin_points[0][0], origin_points[0][1], origin_points[0][2]
            )
            sphere_max_config = cart_to_sphere(
                origin_points[1][0], origin_points[1][1], origin_points[1][2]
            )
            degrees_min = math.degrees(sphere_min_config[2]) - offset + 90
            degrees_max = math.degrees(sphere_max_config[2]) - offset + 90

            if degrees_min > -fov and degrees_max < fov:
                intensity = 1
            histogram_image_min[image_width - i - 1][j] = degrees_min
            histogram_image_max[image_width - i - 1][j] = degrees_max
            histogram_image_height[image_width - i - 1][j] = intensity

    plt.figure()
    plt.title("Najniższy punkt w polu widzenia")
    plt.xlabel("Oś Y [m]", fontsize=8)
    plt.ylabel("Oś X [m]", fontsize=8)

    extent_values = [-2, 2, 0, 5]
    plt.imshow(histogram_image_min, extent=extent_values)
    # plt.xticks(np.arange(-2, 3, 1))
    plt.grid()
    plt.colorbar()

    plt.figure()
    plt.title("Najwyższy punkt w polu widzenia")
    plt.xlabel("Oś Y [m]", fontsize=8)
    plt.ylabel("Oś X [m]", fontsize=8)
    plt.imshow(histogram_image_max, extent=extent_values)
    plt.grid()
    plt.colorbar()

    plt.figure()
    plt.title("Pozycje, w których czujnik widzi w całości wspornik")
    plt.xlabel("Oś Y [m]", fontsize=8)
    plt.ylabel("Oś X [m]", fontsize=8)
    plt.imshow(histogram_image_height, extent=extent_values)
    plt.grid()
    plt.colorbar()

    plt.show()


show_2D_XY_histogram(0.1, 4.0, 5.0)
