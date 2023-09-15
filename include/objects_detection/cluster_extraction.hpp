#pragma once

#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
 #include <pcl/segmentation/sac_segmentation.h>
 #include <pcl/segmentation/extract_clusters.h>

#include <objects_detection/utils.hpp>
class ClusterExtraction {
   public:
    ClusterExtraction() = default;
    std::vector<CloudIPtr> euclidean(CloudPtr cloud, double tolerance, std::size_t min_size, std::size_t max_size) const;
};