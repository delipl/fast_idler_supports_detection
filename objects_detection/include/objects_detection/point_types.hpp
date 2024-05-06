#pragma once
#define PCL_NO_PRECOMPILE
#define PCL_XYZ_POINT_TYPES              \
    PCL_XYZ_POINT_TYPES(pcl::PointXYZIR) \
    (pcl::PointXYZIRL)
#include <pcl/point_types.h>

namespace pcl {
struct PointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    friend std::ostream& operator<<(std::ostream& os, const PointXYZIR& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring,
                                                                                                       ring))

namespace pcl {
struct PointXYZIRL {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    uint16_t label;
    friend std::ostream& operator<<(std::ostream& os, const PointXYZIR& p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(uint16_t, ring,
                                                                                     ring)(uint16_t, label, label))
