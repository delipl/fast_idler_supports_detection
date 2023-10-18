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

TEST(PCDFileTest, LoadPCDFile) {
    CloudPtr cloud(new Cloud);
    auto load_result = load_test_cloud(cloud);
    ASSERT_EQ(load_result, 0);
    (void)load_result;
    myV a(1, 2,3);
    myV b(-1,-2,-3);
    EXPECT_EQ(a-b, {0, 0, 0});

    EXPECT_EQ(cloud->size(), (std::size_t)5669);
}

TEST(RemovingOutliersTest, ThreeVariants) {
    CloudPtr cloud(new Cloud);
    auto load_result = load_test_cloud(cloud);
    ASSERT_EQ(load_result, 0);
    (void)load_result;

    OutlierRemoval algorythm;
    auto cloud_test_1 = algorythm.statistical_pcl(cloud, 20, -0.01);
    save_cloud("statistical_removing_outliers_20_m0.01.pcd", cloud_test_1);
    EXPECT_THAT(cloud_test_1->points.size(), testing::Le(cloud->points.size()));

    auto cloud_test_2 = algorythm.statistical_pcl(cloud, 50, -0.01);
    save_cloud("statistical_removing_outliers_50_m0.01.pcd", cloud_test_2);
    EXPECT_THAT(cloud_test_2->points.size(), testing::Le(cloud->points.size()));

    auto cloud_test_3 = algorythm.statistical_pcl(cloud, 20, -0.05);
    save_cloud("statistical_removing_outliers_20_m0.05.pcd", cloud_test_3);
    EXPECT_THAT(cloud_test_3->points.size(), testing::Le(cloud->points.size()));
}

TEST(OnlyGroundRemoval, DummyThreeVariants) {
    CloudPtr cloud(new Cloud);
    auto load_result = load_test_cloud(cloud);
    ASSERT_EQ(load_result, 0);
    (void)load_result;

    // GroundRemoval ground_removal_argorythm;
    // auto cloud_test_1 = ground_removal_argorythm.dummy(cloud, 0.1);
    // save_cloud("dummy_ground_removal_0.1.pcd", cloud_test_1);

    // auto cloud_test_2 = ground_removal_argorythm.dummy(cloud, 0.3);
    // save_cloud("dummy_ground_removal_0.3.pcd", cloud_test_2);

    // auto cloud_test_3 = ground_removal_argorythm.dummy(cloud, 0.5);
    // save_cloud("dummy_ground_removal_0.5.pcd", cloud_test_3);
}

TEST(OnlyGroundRemoval, GroundPlaneRemovalThreeVariants) {
    CloudPtr cloud(new Cloud);
    auto load_result = load_test_cloud(cloud);
    ASSERT_EQ(load_result, 0);
    (void)load_result;

    auto rotated_cloud = rotate(cloud, 0.0, 0.15, 0.0);
    GroundRemoval ground_removal_argorythm;
    auto cloud_test_1 = ground_removal_argorythm.planar_segmentation(rotated_cloud, 0.15, 0.15);
    save_cloud("plane_ground_removal_0.15_0.15.pcd", cloud_test_1);
    EXPECT_THAT(cloud_test_1->points.size(), testing::Le(cloud->points.size()));

    auto cloud_test_2 = ground_removal_argorythm.planar_segmentation(rotated_cloud, 0.15, 0.01);
    save_cloud("plane_ground_removal_0.15_0.01.pcd", cloud_test_2);
    EXPECT_THAT(cloud_test_2->points.size(), testing::Le(cloud->points.size()));

    auto cloud_test_3 = ground_removal_argorythm.planar_segmentation(rotated_cloud, 0.05, 0.15);
    save_cloud("plane_ground_removal_0.05_0.15.pcd", cloud_test_3);
    EXPECT_THAT(cloud_test_3->points.size(), testing::Le(cloud->points.size()));

    auto cloud_test_4 = ground_removal_argorythm.planar_segmentation(rotated_cloud, 0.05, 0.01);
    save_cloud("plane_ground_removal_0.05_0.01.pcd", cloud_test_4);
    EXPECT_THAT(cloud_test_4->points.size(), testing::Le(cloud->points.size()));
}

TEST(TrainModel, TrainAsTest) {
    // CloudPtr cloud(new Cloud);
    // CloudIPtr cloudi(new CloudI);

    // auto path = path_to_pcds + std::string("train_foot.pcd");
    // pcl::io::loadPCDFile<PointI>(path, *cloudi);
    // for (const auto& point : cloudi->points) {
    //     Point p;
    //     p.x = point.x;
    //     p.y = point.y;
    //     p.z = point.z;
    //     cloud->points.push_back(p);
    // }
//     // cloud->height = 1;
//     // cloud->width = cloud->points.size();
//     CloudPtr cloud(new Cloud);
// load_test_cloud(cloud, "ism_train_cat.pcd"); 
//     Recognition recognition;
//     CloudPtrs clouds;
//     clouds.push_back(cloud);
//     recognition.set_training_data(clouds, 30.0);
//     recognition.train_model(25.0, 2.0);
//     CloudPtr cloud2(new Cloud);
// load_test_cloud(cloud, "ism_test_cat.pcd"); 
//     auto stronges_peaks = recognition.find_objects(cloud2, 25.0);
//     save_cloud("stronges_peaks.pcd", stronges_peaks);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    path_to_pcds = ament_index_cpp::get_package_share_directory("objects_detection");
    path_to_pcds += std::string("/test_data/");
    return RUN_ALL_TESTS();
}
