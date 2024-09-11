import matplotlib.pyplot as plt
import numpy as np
import math

def cartesian_to_spherical(x, y, z):
    r = np.sqrt(x**2 + y**2 + z**2)

    theta = np.arccos(z / r)

    phi = np.arctan2(y, x)

    return r, theta, phi


def beam(x, y, z):
    z0 = 1.45
    y0 = 0
    x0 = 0  # 0.41
    th0 = 0.454

    dth = math.radians(2)  # angle between beams

    v = [math.sin(th0), 0, math.cos(th0)]
    u = [x - x0, y - y0, z - z0]
    unorm = math.sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2])

    sp_uv = v[0] * u[0] + v[1] * u[1] + v[2] * u[2]

    alpha = 3.1415 / 2 - math.acos(sp_uv / unorm)

    if sp_uv < 0:
        k = alpha / dth
    else:
        k = alpha / dth

    return k


def f(x, y):
    lb = beam(x, y, 0 + 0.05)
    hb = beam(x, y, 0.7)
    if lb < -8:
        lb = -8
    if lb > 8:
        val = 0
    if hb > 8:
        hb = 8
    if hb < -8:
        val = 0
    if hb < lb:
        val = 0
    else:
        val = hb - lb + 1

    return val


def add_levels_to_plot(ax):
    # Generate x and y values
    x_values = np.linspace(-3, 9, 150)
    y_values = np.linspace(-8, 8, 150)

    # Create a grid of (x, y) values
    X, Y = np.meshgrid(x_values, y_values)

    # Evaluate the function at each point on the grid for all array
    # Z = f(X, Y)

    # Initialize an array for function values
    Z = np.zeros_like(X)

    # Calculate function values for each (x, y) pair
    for i in range(len(x_values)):
        for j in range(len(y_values)):
            Z[j, i] = f(X[j, i], Y[j, i])

    # Plot the contour lines
    contour = ax.contourf(X, Y, Z, levels=np.arange(1, 9, 1), cmap="coolwarm")
    # Add labels with function values to the contour lines
    ax.clabel(contour, inline=True, fontsize=8, fmt="%1.0f")
    plt.colorbar(contour, label='f(x, y)')
