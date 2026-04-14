#ifndef LEG_KILO_SENSOR_TYPES_H
#define LEG_KILO_SENSOR_TYPES_H

#include <sensor_msgs/msg/imu.hpp>
#include <deque>
#include <string>

#include "pcl_types.h"

namespace legkilo {
namespace common {

struct LidarScan {
    double lidar_begin_time_;
    double lidar_end_time_;
    CloudPtr cloud_;
};

// leg order: FR FL RR RL
struct KinImuMeas {
    double time_stamp_;
    double foot_pos_[4][3];
    double foot_vel_[4][3];
    bool contact_[4];
    double acc_[3];
    double gyr_[3];
};

struct MeasGroup {
    LidarScan lidar_scan_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imus_;
    std::deque<KinImuMeas> kin_imus_;
};

enum class Mode {
    Slam,
    OdomOnly
};

// 统一模式字符串解析逻辑，避免 RosInterface 与 KILO 各自维护一份转换分支。
inline Mode parseMode(const std::string& mode) {
    return mode == "odom_only" ? Mode::OdomOnly : Mode::Slam;
}

enum class OdomFreq {
    Lidar,
    Imu
};

inline OdomFreq parseOdomFreq(const std::string& freq) {
    return freq == "imu" ? OdomFreq::Imu : OdomFreq::Lidar;
}

enum class LidarType { VEL = 1, OUSTER = 2, HESAI = 3, Robosense = 4, UTLidar = 5 };

}  // namespace common
}  // namespace legkilo

#endif  // LEG_KILO_SENSOR_TYPES_H
