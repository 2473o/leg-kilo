#include "interface/ros2/ros_interface.h"

#include <glog/logging.h>
#include <iomanip>
#include <iostream>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common/timer_utils.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/KILO.h"
#include "preprocess/kinematics.h"
#include "preprocess/lidar_processing.h"

namespace legkilo {

#define THREAD_SLEEP(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

RosInterface::RosInterface(const rclcpp::NodeOptions& options) 
    : Node("leg_kilo_node", options) {
    
    RCLCPP_INFO(this->get_logger(), "Ros Interface is being Constructed");

    // QoS 设置：在 ROS 2 中，10000 的深度通常对应 KeepLast
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)); // 适当减小深度以节省内存

    pub_odom_world_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", qos);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/path", qos);
    pub_pointcloud_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", qos);
    pub_pointcloud_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", qos);
    
    if (pub_joint_tf_enable_) {
        pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    }

    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_world_.header.frame_id = "camera_init";
    odom_world_.child_frame_id = "base_link";
    path_world_.header.frame_id = "camera_init";
    path_world_.header.stamp = this->get_clock()->now();
    pose_path_.header.frame_id = "camera_init";
}

RosInterface::~RosInterface() {
    RCLCPP_INFO(this->get_logger(), "Ros Interface is being Destructed");
    
    if (lidar_thread_ && lidar_thread_->joinable()) lidar_thread_->join();
    if (imu_thread_ && imu_thread_->joinable()) imu_thread_->join();
    if (kinematic_thread_ && kinematic_thread_->joinable()) kinematic_thread_->join();
}

bool RosInterface::initParamAndReset(const std::string& config_file) {
    YamlHelper yaml_helper(config_file);

    /* Topic and options*/
    options::kLidarTopic = yaml_helper.get<std::string>("lidar_topic");
    options::kImuUse = yaml_helper.get<bool>("only_imu_use", true);
    options::kKinAndImuUse = static_cast<bool>(!options::kImuUse);
    options::kRedundancy = yaml_helper.get<bool>("redundancy", false);
    
    dynamic_time_adjust_enable_ = yaml_helper.get<bool>("dynamic_time_adjust_enable", false);
    lidar_time_offset_ = 0.0;
    time_offset_calculated_ = false;
    first_lidar_time_ = 0.0;
    first_kin_imu_time_ = 0.0;
    
    if (options::kImuUse) { options::kImuTopic = yaml_helper.get<std::string>("imu_topic"); }
    if (options::kKinAndImuUse) { 
        options::kKinematicTopic = yaml_helper.get<std::string>("kinematic_topic");
        options::kKinematicType = yaml_helper.get<std::string>("kinematic_type");
    }

    /* 模块初始化 */
    kilo_ = std::make_unique<KILO>(config_file);

    /* kinematics*/
    Kinematics::Config kinematics_config;
    kinematics_config.leg_offset_x = yaml_helper.get<double>("leg_offset_x");
    kinematics_config.leg_offset_y = yaml_helper.get<double>("leg_offset_y");
    kinematics_config.leg_calf_length = yaml_helper.get<double>("leg_calf_length");
    kinematics_config.leg_thigh_length = yaml_helper.get<double>("leg_thigh_length");
    kinematics_config.leg_thigh_offset = yaml_helper.get<double>("leg_thigh_offset");
    kinematics_config.contact_force_threshold_up = yaml_helper.get<double>("contact_force_threshold_up");
    kinematics_config.contact_force_threshold_down = yaml_helper.get<double>("contact_force_threshold_down");
    kinematics_ = std::make_unique<Kinematics>(kinematics_config);

    /* lidar processing*/
    LidarProcessing::Config lidar_process_config;
    lidar_process_config.blind_ = yaml_helper.get<float>("blind");
    lidar_process_config.filter_num_ = yaml_helper.get<int>("filter_num");
    lidar_process_config.time_scale_ = yaml_helper.get<double>("time_scale");
    lidar_process_config.point_stamp_correct_ = yaml_helper.get<bool>("point_stamp_correct", false);
    lidar_process_config.lidar_type_ = static_cast<common::LidarType>(yaml_helper.get<int>("lidar_type"));
    lidar_processing_ = std::make_unique<LidarProcessing>(lidar_process_config);

    pub_joint_tf_enable_ = yaml_helper.get<bool>("pub_joint_tf_enable");

    const bool save_traj_enable = yaml_helper.get<bool>("save_traj_enable", false);
    if (save_traj_enable) { traj_saver_ = std::make_unique<TrajectorySaver>(); }

    const bool save_pcd_enable = yaml_helper.get<bool>("save_pcd_enable", false);
    if (save_pcd_enable) { 
        pcd_saver_ = std::make_unique<PcdSaver>(
            yaml_helper.get<int>("pcd_frames_per_file", 100),
            yaml_helper.get<double>("pcd_voxel_leaf_size", 0.1)
        ); 
    }

    return true;
}

void RosInterface::init(const std::string& config_file) {
    this->initParamAndReset(config_file);
    
    this->subscribeLidar();
    if (options::kImuUse) { this->subscribeImu(); }
    if (options::kKinAndImuUse) { this->subscribeKinematicImu(); }
}

void RosInterface::subscribeLidar() {
    lidar_thread_ = std::make_unique<std::thread>(&RosInterface::lidarLoop, this);
}

void RosInterface::subscribeImu() {
    imu_thread_ = std::make_unique<std::thread>(&RosInterface::imuLoop, this);
}

void RosInterface::subscribeKinematicImu() {
    kinematic_thread_ = std::make_unique<std::thread>(&RosInterface::kinematicImuLoop, this);
}


void RosInterface::lidarLoop() {
    auto group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = group;

    sub_lidar_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        options::kLidarTopic, 10, 
        std::bind(&RosInterface::lidarCallBack, this, std::placeholders::_1), 
        sub_opt);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(group, this->get_node_base_interface());
    while (rclcpp::ok() && !options::FLAG_EXIT.load()) {
        executor.spin_some(std::chrono::milliseconds(10));
    }
}

void RosInterface::imuLoop() {
    auto group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = group;

    sub_imu_raw_ = this->create_subscription<sensor_msgs::msg::Imu>(
        options::kImuTopic, 100, 
        std::bind(&RosInterface::imuCallBack, this, std::placeholders::_1), 
        sub_opt);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(group, this->get_node_base_interface());
    while (rclcpp::ok() && !options::FLAG_EXIT.load()) {
        executor.spin_some(std::chrono::milliseconds(5));
    }
}

void RosInterface::kinematicImuLoop() {
    auto group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = group;

    // sub_kinematic_raw_ = this->create_subscription<unitree_legged_msgs::msg::HighState>(
    // options::kKinematicTopic, 100, 
    // std::bind(&RosInterface::kinematicImuCallBack, this, std::placeholders::_1), 
    // sub_opt);

    sub_kinematic_raw_ = this->create_subscription<go2_driver::msg::LegSensor>(
        options::kKinematicTopic, 100, 
        std::bind(&RosInterface::kinematicImuCallBack, this, std::placeholders::_1), 
        sub_opt);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(group, this->get_node_base_interface());
    while (rclcpp::ok() && !options::FLAG_EXIT.load()) {
        executor.spin_some(std::chrono::milliseconds(5));
    }
}

void RosInterface::lidarCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    static double last_scan_time = timestamp;
    
    if (dynamic_time_adjust_enable_) {
        if (first_lidar_time_ == 0.0) {
            first_lidar_time_ = timestamp;
            LOG(INFO) << "[TimeSync] First lidar timestamp: " << std::fixed << std::setprecision(6)
                      << first_lidar_time_;
            if (first_kin_imu_time_ > 0.0 && !time_offset_calculated_) {
                lidar_time_offset_ = first_kin_imu_time_ - first_lidar_time_;
                time_offset_calculated_ = true;
                LOG(INFO) << "[TimeSync] Calculated lidar_time_offset: " << lidar_time_offset_
                          << "s (kin_imu - lidar)";
            }
        }
    }

    Timer::measure("Lidar Processing", [&, this]() {
        if (timestamp < last_scan_time) {
            RCLCPP_WARN(this->get_logger(), "Time inconsistency detected in Lidar data stream");
            lidar_cache_.clear();
        }

        common::LidarScan lidar_scan;
        lidar_processing_->processing(msg, lidar_scan);
        
        if (dynamic_time_adjust_enable_ && time_offset_calculated_) {
            lidar_scan.lidar_end_time_ += lidar_time_offset_;
            lidar_scan.lidar_begin_time_ += lidar_time_offset_;
        }
        
        lidar_cache_.push_back(lidar_scan);
        last_scan_time = timestamp;
    });

    last_scan_time = timestamp;
}

void RosInterface::imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg) {
    static sensor_msgs::msg::Imu last_imu_msg;
    
    if (options::kRedundancy) {
        if (msg->linear_acceleration.z == last_imu_msg.linear_acceleration.z &&
            msg->angular_velocity.z == last_imu_msg.angular_velocity.z) {
            return;
        }
    }

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamp < last_timestamp_imu_) {
            RCLCPP_WARN(this->get_logger(), "Time inconsistency detected in Imu data stream");
            imu_cache_.clear();
        }

        imu_cache_.push_back(msg);
        last_imu_msg = *msg;
        last_timestamp_imu_ = timestamp;
    }
}

void RosInterface::kinematicImuCallBack(const go2_driver::msg::LegSensor::SharedPtr msg) {
    static go2_driver::msg::LegSensor last_highstate_msg;

    if (options::kRedundancy) {
        if (msg->imu_state.accelerometer[2] == last_highstate_msg.imu_state.accelerometer[2] &&
            msg->imu_state.gyroscope[2] == last_highstate_msg.imu_state.gyroscope[2]) {
            return;
        }
    }

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (dynamic_time_adjust_enable_) {
            if (first_kin_imu_time_ == 0.0) {
                first_kin_imu_time_ = timestamp;
                LOG(INFO) << "[TimeSync] First kin_imu timestamp: " << std::fixed << std::setprecision(6)
                          << first_kin_imu_time_;
                if (first_lidar_time_ > 0.0 && !time_offset_calculated_) {
                    lidar_time_offset_ = first_kin_imu_time_ - first_lidar_time_;
                    time_offset_calculated_ = true;
                    LOG(INFO) << "[TimeSync] Calculated lidar_time_offset: " << lidar_time_offset_
                              << "s (kin_imu - lidar)";
                }
            }
        }
        
        if (timestamp < last_timestamp_kin_imu_) {
            RCLCPP_WARN(this->get_logger(), "Time inconsistency detected in Kin. Imu data stream");
            kin_imu_cache_.clear();
        }

        common::KinImuMeas kin_imu_meas;
        kinematics_->processing(*msg, kin_imu_meas);
        kin_imu_cache_.push_back(kin_imu_meas);
        last_timestamp_kin_imu_ = timestamp;
        last_highstate_msg = *msg;
    }

    if (pub_joint_tf_enable_) {
        static std::vector<std::string> joint_names = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
        
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = msg->header.stamp;
        joint_state.name = joint_names;
        for (int i = 0; i < 12; ++i) {
            joint_state.position.push_back(msg->q[i]);
            joint_state.velocity.push_back(msg->dq[i]);
        }
        pub_joint_state_->publish(joint_state);
    }
}

bool RosInterface::syncPackage() {
    static bool lidar_push_ = false;
    static bool caches_cleared_after_sync_ = false;
    std::lock_guard<std::mutex> lk(mutex_);

    if (options::kImuUse) {
        if (lidar_cache_.empty() || imu_cache_.empty()) return false;
        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }
        if (last_timestamp_imu_ < lidar_end_time_) return false;

        measure_.imus_.clear();
        while (!imu_cache_.empty()) {
            double imu_time = rclcpp::Time(imu_cache_.front()->header.stamp).seconds();
            if (imu_time > lidar_end_time_) break;
            measure_.imus_.push_back(imu_cache_.front());
            imu_cache_.pop_front();
        }
        lidar_cache_.pop_front();
        lidar_push_ = false;
        return true;
    }

    if (options::kKinAndImuUse) {
        if (dynamic_time_adjust_enable_) {
            if (!time_offset_calculated_) {
                return false;
            }
            if (!caches_cleared_after_sync_) {
                LOG(INFO) << "[TimeSync] Clearing caches after time offset calculation. "
                          << "lidar_cache=" << lidar_cache_.size()
                          << ", kin_imu_cache=" << kin_imu_cache_.size();
                lidar_cache_.clear();
                kin_imu_cache_.clear();
                lidar_push_ = false;
                caches_cleared_after_sync_ = true;
                return false;
            }
        }
        
        if (lidar_cache_.empty() || kin_imu_cache_.empty()) return false;
        
        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }
        
        if (last_timestamp_kin_imu_ < lidar_end_time_) {
            return false;
        }

        measure_.kin_imus_.clear();
        
        while (!kin_imu_cache_.empty()) {
            if (kin_imu_cache_.front().time_stamp_ > lidar_end_time_) break;
            measure_.kin_imus_.push_back(kin_imu_cache_.front());
            kin_imu_cache_.pop_front();
        }
        lidar_cache_.pop_front();
        lidar_push_ = false;
        
        static int sync_count = 0;
        if (sync_count < 5) {
            LOG(INFO) << "[SyncPackage] Synced! kin_imus_count=" << measure_.kin_imus_.size();
            sync_count++;
        }
        return true;
    }
    return false;
}

void RosInterface::publishOdomTFPath(double end_time) {
    auto ros_time = rclcpp::Time(static_cast<uint64_t>(end_time * 1e9));

    // Odometry
    odom_world_.header.stamp = ros_time;
    odom_world_.pose.pose.position.x = kilo_->getPos()(0);
    odom_world_.pose.pose.position.y = kilo_->getPos()(1);
    odom_world_.pose.pose.position.z = kilo_->getPos()(2);
    q_eigen_ = Eigen::Quaterniond(kilo_->getRot());
    odom_world_.pose.pose.orientation.w = q_eigen_.w();
    odom_world_.pose.pose.orientation.x = q_eigen_.x();
    odom_world_.pose.pose.orientation.y = q_eigen_.y();
    odom_world_.pose.pose.orientation.z = q_eigen_.z();
    pub_odom_world_->publish(odom_world_);

    // TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = ros_time;
    t.header.frame_id = "camera_init";
    t.child_frame_id = "base_link";
    t.transform.translation.x = odom_world_.pose.pose.position.x;
    t.transform.translation.y = odom_world_.pose.pose.position.y;
    t.transform.translation.z = odom_world_.pose.pose.position.z;
    t.transform.rotation = odom_world_.pose.pose.orientation;
    tf_br_->sendTransform(t);

    // Path
    pose_path_.header.stamp = ros_time;
    pose_path_.pose = odom_world_.pose.pose;
    path_world_.poses.push_back(pose_path_);
    pub_path_->publish(path_world_);
}

void RosInterface::publishPointcloudWorld(double end_time) {
    sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl::toROSMsg(*cloud_down_world_, pcl_msg);
    pcl_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(end_time * 1e9));
    pcl_msg.header.frame_id = "camera_init";
    pub_pointcloud_world_->publish(pcl_msg);
}

void RosInterface::publishPointcloudBody(double end_time) {
    if (cloud_down_body_ && !cloud_down_body_->points.empty()) {
        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl::toROSMsg(*cloud_down_body_, pcl_msg);
        pcl_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(end_time * 1e9));
        pcl_msg.header.frame_id = "base_link";
        pub_pointcloud_body_->publish(pcl_msg);
    }
}

void RosInterface::runReset() {
    cloud_raw_.reset(new PointCloudType());
    cloud_down_body_.reset(new PointCloudType());
    cloud_down_world_.reset(new PointCloudType());
    success_pts_size = 0;
}

void RosInterface::run() {
    if (!this->syncPackage()) return;
    this->runReset();

    cloud_raw_ = measure_.lidar_scan_.cloud_;
    double end_time = measure_.lidar_scan_.lidar_end_time_;
    
    if (!kilo_->process(measure_, cloud_down_body_, cloud_down_world_, success_pts_size)) {
        RCLCPP_WARN(this->get_logger(), "KILO processing failed");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "pcl raw size: %zu  pcl down size: %zu",
                cloud_raw_->points.size(), cloud_down_body_->points.size());
    RCLCPP_INFO(this->get_logger(), "useful pcl percent: %.2f %%",
                100.0 * static_cast<double>(success_pts_size) / cloud_down_body_->points.size());

    this->publishOdomTFPath(end_time);
    this->publishPointcloudWorld(end_time);
    this->publishPointcloudBody(end_time);

    if (traj_saver_) { traj_saver_->write(end_time, kilo_->getRot(), kilo_->getPos()); }
    if (pcd_saver_) { pcd_saver_->save(cloud_down_world_); }
}

}  // namespace legkilo
