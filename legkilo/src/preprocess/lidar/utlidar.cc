#include "preprocess/lidar_processing.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace legkilo {

void LidarProcessing::utlidar_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                       common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());

    pcl::PointCloud<utlidar_ros::Point> cloud_raw;
    pcl::fromROSMsg(*msg, cloud_raw);

    const int cloud_size = cloud_raw.points.size();
    if (cloud_size == 0) return;

    lidar_scan.cloud_->points.reserve(cloud_size);

    const double header_time = rclcpp::Time(msg->header.stamp).seconds();

    float first_point_time = 0.0f;
    float last_point_time = 0.0f;
    bool found_first = false;

    for (int i = 0; i < cloud_size; ++i) {
        if (std::isfinite(cloud_raw.points[i].time)) {
            if (!found_first) {
                first_point_time = cloud_raw.points[i].time;
                found_first = true;
            }
            last_point_time = cloud_raw.points[i].time;
        }
    }

    first_point_time *= config_.time_scale_;
    last_point_time *= config_.time_scale_;

    // 计算扫描持续时间
    float scan_duration = last_point_time - first_point_time;
    if (scan_duration < 0.001f) {
        scan_duration = 1.0f / 15.0f;  // 默认 15Hz
    }
    
    lidar_scan.lidar_end_time_ = header_time;
    lidar_scan.lidar_begin_time_ = header_time - scan_duration;
    
    for (int i = 0; i < cloud_size; ++i) {
        if (i % config_.filter_num_) continue;
        if (blindCheck(cloud_raw.points[i])) continue;
        
        PointType added_point;
        added_point.x = cloud_raw.points[i].x;
        added_point.y = cloud_raw.points[i].y;
        added_point.z = cloud_raw.points[i].z;
        added_point.intensity = cloud_raw.points[i].intensity;
        
        float cur_point_time = config_.time_scale_ * cloud_raw.points[i].time;
        // curvature 存储相对于扫描开始的时间偏移
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0f) / 500.0f;
        
        lidar_scan.cloud_->points.push_back(added_point);
    }
}


}  // namespace legkilo
