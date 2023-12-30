#include <pcl/filters/filter_indices.h>  // for pcl::removeNaNFromPointCloud
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <vector>
int main(int argc, char const *argv[]) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("ism_test_converoy8.pcd", *cloud) == -1) {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    pcl::MinCutSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud(cloud);
    seg.setIndices(indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ point;
    point.x = 0;
    point.y = 0;
    point.z = 0.57;
    foreground_points->points.push_back(point);
    seg.setForegroundPoints(foreground_points);

    seg.setSigma(std::stod(argv[1]));
    seg.setRadius(std::stod(argv[2]));
    seg.setNumberOfNeighbours(std::stoi(argv[3]));
    seg.setSourceWeight(std::stod(argv[4]));

    std::vector<pcl::PointIndices> clusters;
    seg.extract(clusters);

    std::cout << "Maximum flow is " << seg.getMaxFlow() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud();
    pcl::visualization::PCLVisualizer viewer("ISM BOXES");

    viewer.addPointCloud(colored_cloud);
    while (!viewer.wasStopped()) {
        viewer.spin();
    }

    return (0);
}
