cmake_minimum_required(VERSION 3.8)
project(objects_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
)

add_executable(object_detection
  src/object_detection.cpp
  src/outlier_removal.cpp
  src/ground_removal.cpp
  src/node.cpp
)
target_link_libraries(object_detection ${PCL_LIBRARIES})

ament_target_dependencies(object_detection rclcpp sensor_msgs pcl_conversions)

install(TARGETS
  object_detection
  DESTINATION lib/${PROJECT_NAME})

ament_package()
