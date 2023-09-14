#include <objects_detection/ground_removal.hpp>

CloudPtr GroundRemoval::dummy(CloudPtr cloud, double threshold) const {
    CloudPtr cloud_without_ground(new Cloud);
    for (auto& point : cloud->points) {
        if (point.z > threshold) {
            cloud_without_ground->points.push_back(point);
        }
    }
    // FIXME: Number of points different than width * height!" thrown in the test body.
    // Nie wiem jak sensownie zmienić wysokość i szerokość

    print_diffrence("ground_removal_dummy", cloud, cloud_without_ground);
    return cloud_without_ground;
}

CloudPtr GroundRemoval::sac_segmentation(CloudPtr cloud, double threshold) const {
    CloudPtr cloud_without_ground(cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (threshold);

    seg.setInputCloud (cloud_without_ground);
    seg.segment (*inliers, *coefficients);
    return cloud_without_ground;
}