#include <objects_detection/ground_removal.hpp>

CloudPtr GroundRemoval::dummy(CloudPtr cloud, double threshold) const {
    CloudPtr cloud_without_ground(new Cloud);
    for (const auto& point : cloud->points) {
        if (point.z > threshold) {
            cloud_without_ground->points.push_back(point);
        }
    }
    cloud_without_ground->height = 1;
    cloud_without_ground->width = cloud_without_ground->points.size();

    return cloud_without_ground;
}

// TODO: ziemia musi być równoległa do osi Z
CloudPtr GroundRemoval::planar_segmentation(CloudPtr cloud, double threshold, double eps_angle) const {
    CloudPtr cloud_without_ground(new Cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<Point> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(400);
    seg.setDistanceThreshold(threshold);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(eps_angle);

    // https://github.com/Auburn-Automow/au_automow_common/blob/920be6a740aa6d738e9954417b41490e353efd04/automow_filtering/src/ground_filter.cc#L164
    bool found_ground = false;
    Cloud filter_pc(*cloud);
    pcl::ExtractIndices<Point> extract;
    while (filter_pc.size() > 10 and not found_ground) {
        seg.setInputCloud(filter_pc.makeShared());
        seg.segment(*inliers, *coefficients);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);

        if (inliers->indices.size() == 0) {
            break;
        } else if (inliers->indices.size() != filter_pc.size()) {
            extract.setInputCloud(filter_pc.makeShared());
            extract.setIndices(inliers);
            extract.setNegative(true);
            Cloud cloud_out2;
            extract.filter(cloud_out2);
            *cloud_without_ground += cloud_out2;
            filter_pc = cloud_out2;
        }
        found_ground = true;
    }

    return cloud_without_ground;
}

CloudPtr GroundRemoval::planar_segmentation(CloudPtr cloud) const {
    return planar_segmentation(cloud, planar_segmentation_threshold, planar_segmentation_eps_angle);
}

CloudPtr GroundRemoval::dummy(CloudPtr cloud) const { return dummy(cloud, dummy_threshold); }