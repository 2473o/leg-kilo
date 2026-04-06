#ifndef LEG_KILO_ROS2_INTERFACE_H
#define LEG_KILO_ROS2_INTERFACE_H

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "common/eigen_types.hpp"
#include "common/pcd_saver.hpp"
#include "common/pcl_types.h"
#include "common/sensor_types.hpp"
#include "common/trajectory_saver.hpp"
#include "interface/ros2/options.h"

// ROS 2 核心头文件
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// TF2 相关
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "go2_driver/msg/leg_sensor.hpp"

namespace legkilo {
class Kinematics;
class LidarProcessing;
class KILO;
}  // namespace legkilo

namespace legkilo {

class RosInterface : public rclcpp::Node {
   public:
    // ROS 2 默认使用 SharedPtr 管理节点
    using SharedPtr = std::shared_ptr<RosInterface>;

    // 构造函数使用 NodeOptions
    RosInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RosInterface();

    void init(const std::string& config_file);
    void run();

   private:
    bool initParamAndReset(const std::string& config_file);
    void subscribeLidar();
    void subscribeKinematicImu();
    void subscribeImu();
    
    void lidarLoop();
    void imuLoop();
    void kinematicImuLoop();

    // ROS 2 回调函数使用 SharedPtr
    void lidarCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg);
    void kinematicImuCallBack(const go2_driver::msg::LegSensor::SharedPtr msg);

    bool syncPackage();
    void runReset();
    void publishOdomTFPath(double end_time);
    void publishPointcloudWorld(double end_time);
    void publishPointcloudBody(double end_time); 

    // ROS 2 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_raw_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_raw_;
    rclcpp::Subscription<go2_driver::msg::LegSensor>::SharedPtr sub_kinematic_raw_;

    // ROS 2 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_world_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_world_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;

    nav_msgs::msg::Odometry odom_world_;
    nav_msgs::msg::Path path_world_;
    
    // TF2 广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
    
    Eigen::Quaterniond q_eigen_;
    geometry_msgs::msg::PoseStamped pose_path_;

    // 子线程
    std::unique_ptr<std::thread> lidar_thread_;
    std::unique_ptr<std::thread> imu_thread_;
    std::unique_ptr<std::thread> kinematic_thread_;

    // 模块
    std::unique_ptr<LidarProcessing> lidar_processing_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<KILO> kilo_;
    std::unique_ptr<TrajectorySaver> traj_saver_;
    std::unique_ptr<PcdSaver> pcd_saver_;

    // 测量缓存（ROS 2 消息改为 SharedPtr）
    std::deque<common::LidarScan> lidar_cache_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_cache_;
    std::deque<common::KinImuMeas> kin_imu_cache_;
    common::MeasGroup measure_;

    std::mutex mutex_;
    double last_timestamp_imu_;
    double last_timestamp_kin_imu_;
    double lidar_end_time_;
    
    // Dynamic time synchronization
    double lidar_time_offset_ = 0.0;
    bool time_offset_calculated_ = false;
    double first_lidar_time_ = 0.0;
    double first_kin_imu_time_ = 0.0;
    bool dynamic_time_adjust_enable_ = false;

    double init_time_ = 0.1;
    bool init_flag_ = true;

    // 修改说明：odom_only 模式下不再依赖接口层缓存原始点云，保留历史成员声明注释以避免直接删除代码痕迹。
    // CloudPtr cloud_raw_;
    CloudPtr cloud_down_body_;
    CloudPtr cloud_down_world_;

    size_t success_pts_size = 0;
    bool pub_joint_tf_enable_ = true;
    
    // 统一使用 common::Mode，避免接口层与核心层枚举定义漂移。
    common::Mode mode_ = common::Mode::Slam;
};

}  // namespace legkilo
#endif  // LEG_KILO_ROS2_INTERFACE_H
