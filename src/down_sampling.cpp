#include <objects_detection/down_sampling.hpp>

CloudPtr DownSampling::leaf(CloudPtr cloud, double size_x, double size_y, double size_z) const {
    pcl::VoxelGrid<Point> vg;
    CloudPtr cloud_filtered (new Cloud);
    vg.setInputCloud (cloud);
    vg.setLeafSize(size_x, size_y, size_z);
    vg.filter(*cloud_filtered);

    print_diffrence("leaf_down_sampling", cloud, cloud_filtered);

    return cloud_filtered;
}
