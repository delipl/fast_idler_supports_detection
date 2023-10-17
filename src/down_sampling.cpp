#include <objects_detection/down_sampling.hpp>

CloudPtr DownSampling::leaf(CloudPtr cloud, double size_x, double size_y, double size_z) const {
    pcl::VoxelGrid<Point> vg;
    CloudPtr cloud_filtered (new Cloud);
    vg.setInputCloud (cloud);
    vg.setLeafSize(size_x, size_y, size_z);
    vg.filter(*cloud_filtered);


    return cloud_filtered;
}

 CloudPtr DownSampling::leaf(CloudPtr cloud) const{
    return leaf(cloud, leaf_size, leaf_size, leaf_size);
 }