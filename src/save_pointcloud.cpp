#include "objects_detection/utils.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudListener : public rclcpp::Node {
   public:
    PointCloudListener(const std::string& topic_name, const std::string& dest_file_name)
        : Node("pointcloud_listener"), topic_name_(topic_name), dest_file_name_(dest_file_name) {
        using namespace std::chrono_literals;

        subscription_ = create_subscription<rclcppCloud>(
            topic_name, 10, [this](const rclcppCloudSharedPtr msg) { pointcloud_callback(msg); });
    }

   private:
    std::string topic_name_;
    std::string dest_file_name_;
    rclcpp::Subscription<rclcppCloud>::SharedPtr subscription_;

    void pointcloud_callback(const rclcppCloudSharedPtr msg) {
        // Konwersja PointCloud2 do chmury punktów PCL
        auto cloud{convert_point_cloud2_to_cloud_ptr(msg)};
        pcl::PCDWriter writer;

        writer.write<Point>(dest_file_name_, *cloud);
        RCLCPP_INFO(this->get_logger(), "PointCloud saved to %s", dest_file_name_.c_str());
        rclcpp::shutdown();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Odczytanie nazwy tematu i ścieżki do pliku z argumentów
    std::string topic_name = "/pointcloud_topic";  // Ustawienie domyślnej nazwy tematu
    std::string dest_file_name = "output.pcd";     // Ustawienie domyślnej nazwy pliku docelowego

    // Sprawdzenie czy podane są argumenty
    if (argc == 3) {
        topic_name = argv[1];
        dest_file_name = argv[2];
    }

    auto pointcloud_listener = std::make_shared<PointCloudListener>(topic_name, dest_file_name);

    rclcpp::spin(pointcloud_listener);

    rclcpp::shutdown();

    return 0;
}
