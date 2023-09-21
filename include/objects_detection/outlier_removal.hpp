#pragma once

#include <pcl/common/distances.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <objects_detection/utils.hpp>

class OutlierRemoval {
   public:
    OutlierRemoval() = default;
    CloudPtr statistical_pcl(CloudPtr cloud, std::size_t k, double stddev_mul_thresh) const;
    CloudPtr statistical_pcl(CloudPtr cloud) const;
    std::size_t statistical_neightbours_count = 0;
    double statistical_stddev_mul_thresh = 0.0;

    CloudPtr radius_outlier(CloudPtr cloud, std::size_t k, double radius) const;
    CloudPtr radius_outlier(CloudPtr cloud) const;
    double radius_outlier_radius = 0.0;
    std::size_t radius_outlier_neightbours_count = 0;
};