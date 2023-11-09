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
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/bounding_box3_d_array.hpp>

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
    double tunnel_length;

    double roll;
    double pitch;
    double yaw;

    double x;
    double y;
    double z;
    double forward_resolution;
    std::size_t forward_histogram_min;
    std::size_t forward_histogram_max;
    std::size_t forward_column_density_threshold;

    double ground_resolution;
    std::size_t ground_histogram_min;
    std::size_t ground_histogram_max;
    double ground_histogram_a;

    std::size_t max_detected_legs = 0;
    std::size_t counter = 0;
    void create_rclcpp_instances();
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<rclcppCloud>::SharedPtr lidar_pc2_sub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr test_pc2_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr mid360_rotated_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr tunneled_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr outlier_removal_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr forward_filtered;
    rclcpp::Publisher<rclcppCloud>::SharedPtr ground_filtered;
    rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_conveyor_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr only_legs_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr merged_density_clouds_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox3DArray>::SharedPtr bounding_box_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr forward_density_histogram_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr forward_density_clustered_histogram_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ground_density_histogram_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ground_density_histogram_multiplied_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ground_density_clustered_histogram_pub_;

    DownSampling down_sampler;
    OutlierRemoval outlier_remover;
    GroundRemoval ground_remover;
    ClusterExtraction clusteler;
    ClusterExtraction clusteler_conveyor;

    CloudPtr remove_points_beyond_tunnel(CloudPtr cloud);
    CloudPtr filter_with_density_on_x_image(CloudPtr cloud, const Histogram &histogram, double resolution, double width,
                                            double height);
    CloudPtr filter_with_density_on_z_image(CloudPtr cloud, const Histogram &histogram, double resolution, double width,
                                            double length);
    CloudPtr remove_far_points_from_ros2bag_converter_bug(CloudPtr cloud, double max_distance);
    CloudPtr merge_clouds(CloudPtrs clouds, double eps);

    Histogram create_histogram(CloudPtr cloud, double resolution,
                                                         double width, double height);
    Histogram remove_low_density_columns(const Histogram &histogram, std::size_t threshold);
    Histogram threshold_histogram(const Histogram &histogram, std::size_t min, std::size_t max);
    Histogram multiply_histogram_by_exp(const Histogram &histogram, double a);

    void save_histogram_to_file(const Histogram &histogram, const std::string & file_name);
    sensor_msgs::msg::Image create_image_from_histogram(const Histogram &histogram);

    MarkersPtr make_markers_from_pointclouds(const CloudIPtrs &clustered_clouds);
    BoundingBoxArrayPtr make_bounding_boxes_from_pointclouds(const CloudIPtrs &clustered_clouds, const std::string &frame_name);

    void save_densities_to_file(const std::vector<std::size_t> &densities, const std::string &path);
};