import matplotlib.pyplot as plt
import numpy as np
import utils
import sys


def plot_3D(title: str, cloud: np.array, cloud1: np.array):
    x = cloud[:, 0]
    y = cloud[:, 1]
    z = cloud[:, 2]
    ll = cloud[:, 4]

    x1 = cloud1[:, 0]
    y1 = cloud1[:, 1]
    z1 = cloud1[:, 2]

    plt.figure("Point Clouds")
    ax = plt.axes(projection="3d")
    plt.grid()
    ax.set_xlabel("X", fontsize=8)
    ax.set_ylabel("Y", fontsize=8)
    ax.set_zlabel("Z", fontsize=8)
    ax.tick_params(axis="both", labelsize=8)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    plt.title(title)

    # Remove points in x1, y1, z1 that are already in x, y, z
    x1_unique, y1_unique, z1_unique, _ = [], [], [], []
    for xi1, yi1, zi1 in zip(x1, y1, z1):
        found = False
        for xi, yi, zi in zip(x, y, z):
            d_x = xi1 - xi
            d_y = yi1 - yi
            d_z = zi1 - zi
            # print(d_x*d_x + d_y*d_y + d_z*d_z)
            if d_x * d_x + d_y * d_y + d_z * d_z < 0.0001:
                found = True
                break
        if not found:
            x1_unique.append(xi1)
            y1_unique.append(yi1)
            z1_unique.append(zi1)
    z = np.array(z)

    ax.scatter3D(
        x,
        y,
        z,
        s=[0.8 for _ in range(len(x))],
        # c="b",
        c=ll,
        marker="o",
        alpha=1,
        depthshade=False,
        cmap="winter",
    )
    z1_unique = np.array(z1_unique)
    ax.scatter3D(
        x1_unique,
        y1_unique,
        z1_unique,
        c=1 - z1_unique,
        s=[0.1 for _ in range(len(x1_unique))],
        marker="o",
        cmap="autumn",
    )
    ax.set_box_aspect([np.ptp(coord) for coord in [x1, y1, z1]])
    ax.view_init(azim=-155, elev=20)

    plt.tight_layout()

    plt.show()


cloud = utils.read_pcd(sys.argv[2])
cloud1 = utils.read_pcd(sys.argv[3])
plot_3D(sys.argv[1], cloud, cloud1)
