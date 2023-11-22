#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <objects_detection/utils.hpp>

#include <objects_detection/object_detection.hpp>

std::string path_to_pcds;

int load_test_cloud(CloudPtr cloud, const char* file_name = "test_cloud.pcd") {
    auto path = path_to_pcds + std::string(file_name);
    return pcl::io::loadPCDFile<Point>(path, *cloud);
}

void save_cloud(const std::string& name, CloudPtr cloud) {
    pcl::PCDWriter writer;
    auto path = path_to_pcds + std::string(name);
    writer.write<Point>(path, *cloud);
}

// TEST(PCDFileTest, LoadPCDFile) {
//     CloudPtr cloud(new Cloud);
//     auto load_result = load_test_cloud(cloud);
//     ASSERT_EQ(load_result, 0);
//     (void)load_result;

//     EXPECT_EQ(cloud->size(), (std::size_t)5669);
// }

// TEST(RemovingOutliersTest, ThreeVariants) {
//     CloudPtr cloud(new Cloud);
//     auto load_result = load_test_cloud(cloud);
//     ASSERT_EQ(load_result, 0);
//     (void)load_result;

//     OutlierRemoval algorythm;
//     auto cloud_test_1 = algorythm.statistical_pcl(cloud, 20, -0.01);
//     save_cloud("statistical_removing_outliers_20_m0.01.pcd", cloud_test_1);
//     EXPECT_THAT(cloud_test_1->points.size(), testing::Le(cloud->points.size()));

//     auto cloud_test_2 = algorythm.statistical_pcl(cloud, 50, -0.01);
//     save_cloud("statistical_removing_outliers_50_m0.01.pcd", cloud_test_2);
//     EXPECT_THAT(cloud_test_2->points.size(), testing::Le(cloud->points.size()));

//     auto cloud_test_3 = algorythm.statistical_pcl(cloud, 20, -0.05);
//     save_cloud("statistical_removing_outliers_20_m0.05.pcd", cloud_test_3);
//     EXPECT_THAT(cloud_test_3->points.size(), testing::Le(cloud->points.size()));
// }

// TEST(OnlyGroundRemoval, DummyThreeVariants) {
//     CloudPtr cloud(new Cloud);
//     auto load_result = load_test_cloud(cloud);
//     ASSERT_EQ(load_result, 0);
//     (void)load_result;

//     // GroundRemoval ground_removal_argorythm;
//     // auto cloud_test_1 = ground_removal_argorythm.dummy(cloud, 0.1);
//     // save_cloud("dummy_ground_removal_0.1.pcd", cloud_test_1);

//     // auto cloud_test_2 = ground_removal_argorythm.dummy(cloud, 0.3);
//     // save_cloud("dummy_ground_removal_0.3.pcd", cloud_test_2);

//     // auto cloud_test_3 = ground_removal_argorythm.dummy(cloud, 0.5);
//     // save_cloud("dummy_ground_removal_0.5.pcd", cloud_test_3);
// }

// TEST(OnlyGroundRemoval, GroundPlaneRemovalThreeVariants) {
//     CloudPtr cloud(new Cloud);
//     auto load_result = load_test_cloud(cloud);
//     ASSERT_EQ(load_result, 0);
//     (void)load_result;

//     auto rotated_cloud = rotate(cloud, 0.0, 0.15, 0.0);
//     GroundRemoval ground_removal_argorythm;
//     auto cloud_test_1 = ground_removal_argorythm.planar_segmentation(rotated_cloud, 0.15, 0.15);
//     save_cloud("plane_ground_removal_0.15_0.15.pcd", cloud_test_1);
//     EXPECT_THAT(cloud_test_1->points.size(), testing::Le(cloud->points.size()));

//     auto cloud_test_2 = ground_removal_argorythm.planar_segmentation(rotated_cloud, 0.15, 0.01);
//     save_cloud("plane_ground_removal_0.15_0.01.pcd", cloud_test_2);
//     EXPECT_THAT(cloud_test_2->points.size(), testing::Le(cloud->points.size()));

//     auto cloud_test_3 = ground_removal_argorythm.planar_segmentation(rotated_cloud, 0.05, 0.15);
//     save_cloud("plane_ground_removal_0.05_0.15.pcd", cloud_test_3);
//     EXPECT_THAT(cloud_test_3->points.size(), testing::Le(cloud->points.size()));

//     auto cloud_test_4 = ground_removal_argorythm.planar_segmentation(rotated_cloud, 0.05, 0.01);
//     save_cloud("plane_ground_removal_0.05_0.01.pcd", cloud_test_4);
//     EXPECT_THAT(cloud_test_4->points.size(), testing::Le(cloud->points.size()));
// }

TEST(PlaneFilter, PCLRANSAC) {
    CloudPtr cloud(new Cloud);
    load_test_cloud(cloud, "filtered_legs_0.pcd");
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setProbability(1.0);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold(0.05);
    // seg.setEpsAngle(0.2);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        // ASSERT();
    }
    CloudPtr plane_cloud(new Cloud);
    for (const auto& idx : inliers->indices){
        Point p = cloud->points[idx];
        plane_cloud->points.push_back(p);
    }
    plane_cloud->width = 1;
    plane_cloud->height = plane_cloud->points.size();

    save_cloud("Test_saved_leg.pcd", plane_cloud);
    EXPECT_THAT(plane_cloud->points.size(), testing::Le(cloud->points.size()));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    path_to_pcds = ament_index_cpp::get_package_share_directory("objects_detection");
    path_to_pcds += std::string("/test_data/");
    return RUN_ALL_TESTS();
}
