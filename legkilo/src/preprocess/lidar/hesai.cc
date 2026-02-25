#include "preprocess/lidar_processing.h"

namespace legkilo {

void LidarProcessing::hesaiHandler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, common::LidarScan& lidar_scan) {
    lidar_scan.cloud_.reset(new PointCloudType());
    pcl::PointCloud<hesai_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);

    double first_point_time = config_.time_scale_ * cloud_pcl_raw.points.front().timestamp;
    double last_point_time = config_.time_scale_ * cloud_pcl_raw.points.back().timestamp;

    lidar_scan.lidar_begin_time_ = first_point_time;
    lidar_scan.lidar_end_time_ = last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);

    for (int i = 0; i < cloud_size; ++i) {
        if ((i % config_.filter_num_) || blindCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        double cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].timestamp;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0f) / 500.0f;

        lidar_scan.cloud_->points.push_back(added_point);
    }
}
}