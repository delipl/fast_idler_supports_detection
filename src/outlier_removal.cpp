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

    print_diffrence("statistical_outlier_removal", cloud, filtered);

    return filtered;
}

CloudPtr OutlierRemoval::statistical_pcl(CloudPtr cloud) const {
    return statistical_pcl(cloud, statistical_neightbours_count, statistical_stddev_mul_thresh);
}

CloudPtr OutlierRemoval::radius_outlier(CloudPtr cloud, std::size_t k, double radius) const {
    pcl::RadiusOutlierRemoval<Point> filter;
    CloudPtr filtered(new Cloud);

    filter.setInputCloud(cloud);
    filter.setRadiusSearch(radius);
    filter.setMinNeighborsInRadius(k);
    filter.setKeepOrganized(true);
    // apply filter
    filter.filter(*filtered);
    print_diffrence("statistical_outlier_removal", cloud, filtered);
    return filtered;
}

CloudPtr OutlierRemoval::radius_outlier(CloudPtr cloud) const {
    return radius_outlier(cloud, radius_outlier_neightbours_count, radius_outlier_radius);
}