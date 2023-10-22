#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <objects_detection/cluster_extraction.hpp>
#include <objects_detection/down_sampling.hpp>
#include <objects_detection/ground_removal.hpp>
#include <objects_detection/outlier_removal.hpp>
#include <objects_detection/utils.hpp>

using namespace std::chrono_literals;

class ObjectDetection : public rclcpp::Node {
   public:
    ObjectDetection();

   private:
    void declare_parameters();
    void get_parameters();

    std::string pointcloud_topic_name;

    double tunnel_width;
    double tunnel_height;

    double roll;
    double pitch;
    double yaw;

    double x;
    double y;
    double z;
    double histogram_resolution;
    std::size_t histogram_min;
    std::size_t histogram_max;
    std::size_t column_density_threshold;

    std::size_t max_detected_legs = 0;

    void create_rclcpp_instances();
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<rclcppCloud>::SharedPtr lidar_pc2_sub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr test_pc2_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr mid360_rotated_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr leaf_down_sampling_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr outlier_removal_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr without_ground_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_conveyor_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr only_legs_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr density_histogram_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr density_clustered_histogram_pub_;

    DownSampling down_sampler;
    OutlierRemoval outlier_remover;
    GroundRemoval ground_remover;
    ClusterExtraction clusteler;
    ClusterExtraction clusteler_conveyor;

    CloudPtr remove_points_beyond_tunnel(CloudPtr cloud);
    CloudPtr filter_with_dencity_on_x_image(CloudPtr cloud, const Histogram &histogram, double resolution);
    Histogram create_histogram(CloudPtr cloud, double resolution);
    Histogram remove_low_density_colums(const Histogram &histogram, std::size_t threshold);
    Histogram threshold_histogram(const Histogram &histogram, std::size_t min, std::size_t max);

    std::vector<std::size_t> count_densities(const Histogram &histogram);
    sensor_msgs::msg::Image create_image_from_histogram(const Histogram &histogram);

    void save_dencities_to_file(const std::vector<std::size_t> &dencities, const std::string &path);
};