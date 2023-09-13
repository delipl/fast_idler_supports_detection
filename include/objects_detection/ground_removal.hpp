#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <objects_detection/utils.hpp>

class GroundRemoval {
   public:
    GroundRemoval() = default;
    CloudPtr dummy(CloudPtr cloud, double threshold) const;
    CloudPtr sac_segmentation(CloudPtr cloud, double threshold) const;
};