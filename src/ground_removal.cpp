#include <objects_detection/ground_removal.hpp>

CloudPtr GroundRemoval::dummy(CloudPtr cloud, double threshold) const {
    CloudPtr cloud_without_ground(new Cloud);
    for (auto& point : cloud->points) {
        if(point.z > threshold){
            cloud_without_ground->points.push_back(point);
        }
    }
    return cloud_without_ground;
}