#include "preprocess/lidar_processing.h"

namespace legkilo {
LidarProcessing::LidarProcessing(LidarProcessing::Config config) : config_(config) {
    LOG(INFO) << "Lidar Processing is Constructed";
    cloud_pcl_.reset(new PointCloudType());
}

LidarProcessing::~LidarProcessing() { LOG(INFO) << "Lidar Processing is Destructed"; }

common::LidarType LidarProcessing::getLidarType() const { return config_.lidar_type_; }

void LidarProcessing::processing(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, common::LidarScan& lidar_scan) {
    switch (config_.lidar_type_) {
        case common::LidarType::VEL: velodyneHandler(msg, lidar_scan); break;

        case common::LidarType::OUSTER: ousterHander(msg, lidar_scan); break;

        case common::LidarType::HESAI: hesaiHandler(msg, lidar_scan); break;

        case common::LidarType::Robosense:  
            {
            double start_time, end_time;
            robosense_handler(msg, lidar_scan, 0, 1, start_time, end_time);
            }
        break;

        default: LOG(ERROR) << " Lidar Type is Not Currently Available"; break;
    }
}

}  // namespace legkilo
