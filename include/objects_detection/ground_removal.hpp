#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <objects_detection/utils.hpp>

class GroundRemoval {
   public:
    GroundRemoval() = default;
    CloudPtr dummy(CloudPtr cloud, double threshold) const;
};