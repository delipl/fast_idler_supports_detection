#pragma once

#include <pcl/common/distances.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <objects_detection/utils.hpp>

class OutlierRemoval {
   public:
    OutlierRemoval() = default;
    CloudPtr statistical_pcl(CloudPtr cloud, uint k, double stddev_mul_thresh) const;
};