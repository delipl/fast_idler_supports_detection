#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <objects_detection/object_detection.hpp>
#include <objects_detection/utils.hpp>
#include <pcl/visualization/pcl_visualizer.h>
std::string path_to_pcds;

int load_test_cloud(CloudPtr cloud, const char* file_name = "test_cloud.pcd") {
    auto path = path_to_pcds + std::string(file_name);
    return pcl::io::loadPCDFile<Point>(path, *cloud);
}
template <typename T>
void save_cloud(const std::string& name, std::shared_ptr<pcl::PointCloud<T>> cloud) {
    pcl::PCDWriter writer;
    auto path = path_to_pcds + std::string(name);
    writer.write<T>(path, *cloud);
}

TEST(ObjectDetectionTest, ToCharts) {
    CloudPtr cloud_raw(new Cloud);
    ObjectDetection object_detection;
    load_test_cloud(cloud_raw, "velodyne_raw_13.pcd");
    auto cloud = object_detection.remove_far_points_from_ros2bag_converter_bug(cloud_raw, 5.0);

    // Ground Filtering
    Eigen::Vector3d normal_vec;
    double ground_height;
    auto filtered_ground_clouds = object_detection.filter_ground_and_get_normal_and_height(
        cloud, pcl::SACMODEL_PLANE, 800, 0.05, std::ref(normal_vec), std::ref(ground_height));
    auto ground = filtered_ground_clouds.first;
    auto without_ground = filtered_ground_clouds.second;

    EXPECT_THAT(ground->points.size(), testing::Le(cloud->points.size()));
    EXPECT_THAT(without_ground->points.size(), testing::Le(cloud->points.size()));
    EXPECT_THAT(ground->points.size(), testing::Le(without_ground->points.size()));
    save_cloud("ground_5.pcd", ground);
    save_cloud("without_ground_5.pcd", without_ground);

    auto aligned_cloud = object_detection.align_to_normal(without_ground, normal_vec, ground_height);
    EXPECT_THAT(aligned_cloud->points.size(), without_ground->points.size());
    save_cloud("aligned_5.pcd", aligned_cloud);

    auto tunneled_cloud = object_detection.remove_points_beyond_tunnel(aligned_cloud);
    EXPECT_THAT(tunneled_cloud->points.size(), testing::Le(aligned_cloud->points.size()));
    save_cloud("tunneled_5.pcd", tunneled_cloud);

    auto histogram = object_detection.create_histogram(tunneled_cloud, 0.1);
    auto threshold_histogram_yz = object_detection.threshold_histogram(histogram, 15, 30);
    auto density_filtered_yz =
        object_detection.filter_with_density_on_x_image(tunneled_cloud, threshold_histogram_yz);
    EXPECT_THAT(density_filtered_yz->points.size(), testing::Le(tunneled_cloud->points.size()));
    save_cloud("density_filtered_yz_5.pcd", density_filtered_yz);

    auto rotated_for_ground_histogram = rotate(tunneled_cloud, 0.0, -M_PI / 2.0, 0.0);
    auto ground_histogram = object_detection.create_histogram(rotated_for_ground_histogram, 0.1);
    auto ground_local_peeks = object_detection.segment_local_peeks(ground_histogram, 10, 3);
    auto density_filtered_xy =
        object_detection.filter_with_density_on_z_image(tunneled_cloud, ground_local_peeks);
    EXPECT_THAT(density_filtered_xy->points.size(), testing::Le(tunneled_cloud->points.size()));
    save_cloud("density_filtered_xy_5.pcd", density_filtered_xy);

    auto merged_density_cloud{object_detection.merge_clouds_and_remove_simillar_points({density_filtered_yz, density_filtered_xy}, 0.0001)};
    EXPECT_THAT(density_filtered_xy->points.size(), testing::Le(merged_density_cloud->points.size()));
    EXPECT_THAT(density_filtered_yz->points.size(), testing::Le(merged_density_cloud->points.size()));
    save_cloud("merged_density_5.pcd", merged_density_cloud);
    pcl::NormalEstimation<Point, pcl::Normal> ne;
    ne.setInputCloud (merged_density_cloud);
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.1);
    ne.compute (*cloud_normals);
    save_cloud("merged_legs_normals.pcd", cloud_normals);
    //     // Inicjalizuj wizualizator PCL
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(merged_density_cloud, cloud_normals);
    
    while (!viewer.wasStopped()) {
        viewer.spin();
    }

    ClusterExtraction cluster;
    cluster.euclidean_tolerance = 0.4;
    cluster.euclidean_min_size = 20;
    cluster.euclidean_max_size = 500;
    CloudIPtrs clustered_legs = cluster.euclidean(merged_density_cloud);

    CloudIPtr merged_legs(new CloudI);
    for (auto& clustered_cloud : clustered_legs) {
        if (clustered_cloud->size()) {
            *merged_legs += *clustered_cloud;
        }
    }
    merged_legs->width = 1;
    merged_legs->height = merged_legs->size();
    save_cloud("merged_legs_5.pcd", merged_legs);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    path_to_pcds = ament_index_cpp::get_package_share_directory("objects_detection");
    path_to_pcds += std::string("/test_data/");
    return RUN_ALL_TESTS();
}
