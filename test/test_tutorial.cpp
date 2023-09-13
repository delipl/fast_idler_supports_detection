#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <objects_detection/object_detection.hpp>
std::string path_to_pcds;

int load_test_cloud(CloudPtr cloud){
    auto path = path_to_pcds + std::string("test_cloud.pcd");
    return pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
}

void save_cloud(const std::string &name, CloudPtr cloud){
    pcl::PCDWriter writer;
    auto path = path_to_pcds + std::string(name);
    writer.write<pcl::PointXYZ>(path, *cloud);
}

TEST(PCDFileTest, LoadPCDFile) {
    CloudPtr cloud(new Cloud);
    auto load_result = load_test_cloud(cloud);
    ASSERT_EQ(load_result, 0);
    (void)load_result;

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

    auto cloud_test_2 = algorythm.statistical_pcl(cloud, 50, -0.01);
    save_cloud("statistical_removing_outliers_50_m0.01.pcd", cloud_test_2);

    auto cloud_test_3 = algorythm.statistical_pcl(cloud, 20, -0.05);
    save_cloud("statistical_removing_outliers_20_m0.05.pcd", cloud_test_3);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    path_to_pcds = ament_index_cpp::get_package_share_directory("objects_detection");
    path_to_pcds += std::string("/test_data/");
    return RUN_ALL_TESTS();
}
