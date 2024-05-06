#include <objects_detection/point_types.hpp>

namespace pcl {
std::ostream& operator<<(std::ostream& os, const pcl::PointXYZIR& p) {
    os << "(" << p.x << "," << p.y << "," << p.z << "," << p.intensity << "," << p.ring << ")";
    return (os);
}

std::ostream& operator<<(std::ostream& os, const pcl::PointXYZIRL& p) {
    os << "(" << p.x << "," << p.y << "," << p.z << "," << p.intensity << "," << p.ring << "," << p.label << ")";
    return (os);
}

}  // namespace pcl
