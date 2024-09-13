from matplotlib.patches import Rectangle, Circle


def plot_feature(ax, features, label, color):
    mean_points_x = []
    mean_points_y = []
    mean_points_z = []
    for feature in features:
        mean_points_x.append(feature.mean_point[0])
        mean_points_y.append(feature.mean_point[1])
        mean_points_z.append(feature.mean_point[2])
    ax.plot(mean_points_x, mean_points_y, ".", c=color, label=label, markersize=5)


def plot_feature_with_bbox(ax, features, label, color, bbox_color):
    mean_points_x = []
    mean_points_y = []
    mean_points_z = []
    for feature in features:
        mean_points_x.append(feature.mean_point[0])
        mean_points_y.append(feature.mean_point[1])
        mean_points_z.append(feature.mean_point[2])
        center = (
            feature.box_center[0],
            feature.box_center[1],
        )
        ax.add_patch(
            Circle(
                center,
                0.2,
                edgecolor=bbox_color,
                facecolor=None,
                alpha=0.08,
            )
        )
        
        center = (
            feature.box_center[0] - feature.box_size[0] / 2,
            feature.box_center[1] - feature.box_size[1] / 2,
        )
        
        ax.add_patch(
            Rectangle(
                center,
                feature.box_size[0],
                feature.box_size[1],
                edgecolor=bbox_color,
                facecolor=None,
                alpha=0.08,
                fill=False
            )
        )
    ax.plot(mean_points_x, mean_points_y, ".", c=color, label=label, markersize=3)