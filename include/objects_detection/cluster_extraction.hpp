#pragma once

#include <pcl/filters/voxel_grid.h>

#include <utils.hpp>
class ClusterExtraction {
   public:
    ClusterExtraction() = default;
    CloudPtr euclidean(CloudPtr cloud) const;
};