#include <iostream>
#include <objects_detection/outlier_removal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

CloudPtr OutlierRemoval::statistical_pcl(CloudPtr cloud, std::size_t k, double stddev_mul_thresh) const {
    pcl::StatisticalOutlierRemoval<Point> filter;
    CloudPtr filtered(new Cloud);
    filter.setInputCloud(cloud);
    filter.setMeanK(k);
    filter.setStddevMulThresh(stddev_mul_thresh);
    filter.filter(*filtered);

    return filtered;
}

CloudPtr OutlierRemoval::statistical_pcl(CloudPtr cloud) const {
    return statistical_pcl(cloud, statistical_neighbors_count, statistical_stddev_mul_thresh);
}

CloudPtr OutlierRemoval::radius_outlier(CloudPtr cloud, std::size_t k, double radius) const {
    pcl::RadiusOutlierRemoval<Point> filter;
    CloudPtr filtered(new Cloud);

    filter.setInputCloud(cloud);
    filter.setRadiusSearch(radius);
    filter.setMinNeighborsInRadius(k);
    filter.setKeepOrganized(false);
    filter.filter(*filtered);

    return filtered;
}

CloudPtr OutlierRemoval::radius_outlier(CloudPtr cloud) const {
    return radius_outlier(cloud, radius_outlier_neighbors_count, radius_outlier_radius);
}

CloudPtr OutlierRemoval::radius_vertical_outlier(CloudPtr cloud, std::size_t k, double distance, double diff) const {
    CloudPtr density_z_cloud(new Cloud);
    for (const auto& p : cloud->points) {
        std::size_t actual_neighbor = 0;
        for (const auto& q : cloud->points) {
            if (pcl::euclideanDistance(p, q) < distance) {
                if (std::abs(p.z - q.z) > diff) {
                    actual_neighbor++;
                }
            }
        }
        if (actual_neighbor >= k) {
            density_z_cloud->points.push_back(p);
        }
    }
    density_z_cloud->width = 1;
    density_z_cloud->height = density_z_cloud->points.size();
    return density_z_cloud;
}

CloudPtr OutlierRemoval::radius_vertical_outlier(CloudPtr cloud) const{
    return radius_vertical_outlier(cloud, radius_vertical_outlier_neighbors_count, radius_vertical_outlier_radius,
                                   radius_vertical_outlier_diff);
}

