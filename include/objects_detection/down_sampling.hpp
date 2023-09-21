#pragma once

#include <pcl/filters/voxel_grid.h>

#include <objects_detection/utils.hpp>

class DownSampling {
   public:
    DownSampling() = default;
    CloudPtr leaf(CloudPtr cloud, double size_x, double size_y, double size_z) const;
    CloudPtr leaf(CloudPtr cloud) const;

    double leaf_size = 0.0;
};