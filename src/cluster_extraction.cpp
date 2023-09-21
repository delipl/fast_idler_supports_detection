#include <objects_detection/cluster_extraction.hpp>

std::vector<CloudIPtr> ClusterExtraction::euclidean(CloudPtr cloud, double tolerance, std::size_t min_size,
                                                    std::size_t max_size) const {
    CloudPtr cloud_filtered(cloud);
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance(tolerance);  // 2cm
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    std::vector<CloudIPtr> clouds;
    for (const auto& cluster : cluster_indices) {
        CloudIPtr cloud_cluster(new CloudI);
        for (const auto& idx : cluster.indices) {
            PointI point;
            point.x = (*cloud_filtered)[idx].x;
            point.y = (*cloud_filtered)[idx].y;
            point.z = (*cloud_filtered)[idx].z;
            point.intensity = j * 10;
            cloud_cluster->push_back(point);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        RCLCPP_INFO_STREAM(rclcpp::get_logger("cluster_extraction"),
                           "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points.");
        clouds.push_back(cloud_cluster);
        ++j;
    }
    return clouds;
}

std::vector<CloudIPtr> ClusterExtraction::euclidean(CloudPtr cloud) const {
    return euclidean(cloud, euclidean_tolerance, euclidean_min_size, euclidean_max_size);
}
