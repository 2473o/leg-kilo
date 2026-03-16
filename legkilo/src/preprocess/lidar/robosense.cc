#include "preprocess/lidar_processing.h"
#include <array>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

namespace legkilo {

/**
 * @brief 检查 PointCloud2 消息是否包含指定字段
 */
static bool hasField(const sensor_msgs::msg::PointCloud2& msg, const std::string& field_name) {
    for (const auto& field : msg.fields) {
        if (field.name == field_name) {
            return true;
        }
    }
    return false;
}

static double normalizePointTimestamp(double ts, double msg_time) {
    const std::array<double, 4> candidates = {ts, ts * 1e-3, ts * 1e-6, ts * 1e-9};
    double best = candidates[0];
    double best_diff = std::abs(candidates[0] - msg_time);
    for (size_t i = 1; i < candidates.size(); ++i) {
        const double diff = std::abs(candidates[i] - msg_time);
        if (diff < best_diff) {
            best_diff = diff;
            best = candidates[i];
        }
    }
    if (best_diff < 3600.0) {
        return best;
    }
    return ts;
}

/**
 * @brief Robosense/通用雷达处理器
 * 
 * 自动检测点云格式：
 * 1. 如果同时有 ring 和 timestamp 字段，使用精确时间戳
 * 2. 否则使用标准 PointXYZI 格式，curvature 设为 0
 * 
 * 参考 Fast-LIO 的 handler_robosense.cpp 实现
 */
void LidarProcessing::robosense_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                      common::LidarScan& lidar_scan,
                                      int i_sub_cloud, int num_sub_cloud, double& start_time, double& end_time) {
    lidar_scan.cloud_.reset(new PointCloudType());

    // 检查是否有 ring 和 timestamp 字段
    const bool has_ring = hasField(*msg, "ring");
    const bool has_timestamp = hasField(*msg, "timestamp");
    
    // 获取消息头时间戳
    const double msg_time = rclcpp::Time(msg->header.stamp).seconds();

    if (has_ring && has_timestamp) {
        // 使用 Robosense 原生格式（包含 ring 和 timestamp 字段）
        pcl::PointCloud<robosense_ros::Point> cloud_robosense;
        pcl::fromROSMsg(*msg, cloud_robosense);
        
        const int cloud_size = cloud_robosense.points.size();
        lidar_scan.cloud_->points.reserve(cloud_size);

        // 从点的 timestamp 获取起止时间
        int points_per_sub = cloud_size / num_sub_cloud;
        int start_idx = i_sub_cloud * points_per_sub;
        int end_idx = (i_sub_cloud + 1) * points_per_sub;
        if (i_sub_cloud == num_sub_cloud - 1) end_idx = cloud_size;
        
        if (start_idx >= cloud_size) return;
        
        start_time = normalizePointTimestamp(cloud_robosense.points[start_idx].timestamp, msg_time);
        end_time = normalizePointTimestamp(cloud_robosense.points[end_idx - 1].timestamp, msg_time);
        
        lidar_scan.lidar_begin_time_ = start_time;
        lidar_scan.lidar_end_time_ = end_time;

        // 处理点云
        for (int i = start_idx; i < end_idx; ++i) {
            const auto& pt = cloud_robosense.points[i];

            // 步进过滤与盲区检查
            if ((i % config_.filter_num_) != 0 || blindCheck(pt)) {
                continue;
            }

            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }

            PointType added_point;
            added_point.x = pt.x;
            added_point.y = pt.y;
            added_point.z = pt.z;
            added_point.intensity = pt.intensity;

            // 使用点的时间戳计算相对时间
            double pt_time = normalizePointTimestamp(pt.timestamp, msg_time);
            added_point.curvature = static_cast<float>((pt_time - start_time) * config_.time_scale_);

            lidar_scan.cloud_->points.push_back(added_point);
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("robosense_handler"), 
                     "Robosense format with timestamp: raw=%d, output=%zu",
                     cloud_size, lidar_scan.cloud_->points.size());
    } else {
        // 使用标准 PointXYZI 格式（无 ring/timestamp 字段）
        // 参考 Fast-LIO: curvature 设为 0，start_time/end_time 使用消息头时间戳
        pcl::PointCloud<pcl::PointXYZI> cloud_xyzi;
        pcl::fromROSMsg(*msg, cloud_xyzi);
        
        const int cloud_size = cloud_xyzi.points.size();
        lidar_scan.cloud_->points.reserve(cloud_size);

        // 使用消息头时间戳作为起止时间
        start_time = msg_time;
        end_time = msg_time;
        
        lidar_scan.lidar_begin_time_ = msg_time;
        lidar_scan.lidar_end_time_ = msg_time;

        // 处理点云
        for (int i = 0; i < cloud_size; ++i) {
            const auto& pt = cloud_xyzi.points[i];

            // 步进过滤与盲区检查
            if ((i % config_.filter_num_) != 0 || blindCheck(pt)) {
                continue;
            }

            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }

            PointType added_point;
            added_point.x = pt.x;
            added_point.y = pt.y;
            added_point.z = pt.z;
            added_point.intensity = pt.intensity;

            // 没有时间戳字段，curvature 设为 0
            added_point.curvature = 0.0f;

            lidar_scan.cloud_->points.push_back(added_point);
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("robosense_handler"), 
                     "PointXYZI format (no timestamp): raw=%d, output=%zu",
                     cloud_size, lidar_scan.cloud_->points.size());
    }
}

} // namespace legkilo
