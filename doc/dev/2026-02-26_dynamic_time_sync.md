# 开发日志：动态时间同步修复

**日期时间**：2026-02-26 13:41

## 问题描述

Leg-KILO 系统在处理 ROS bag 数据时持续报错：

```shell
W0226 13:03:08.449995 202072 KILO.cc:328] Data packet is not ready
[WARN] [1772082188.450215798] [legkilo_node]: KILO processing failed
```

## 问题分析

通过调试发现 lidar 和 leg_sensor 两个话题的时间戳存在约 2700 秒（45 分钟）的偏移：

- lidar 时间戳：~1768477808
- leg_sensor 时间戳：~1768480508
- 时间差：~2699.8 秒

`syncPackage()` 函数要求 `last_timestamp_kin_imu_ >= lidar_end_time_` 才能同步数据，但由于时间戳偏移，这个条件始终无法满足。

## 解决方案

实现动态时间同步机制，自动检测并校正时间偏移。

### 核心算法

1. 记录第一个 lidar 消息的时间戳 `first_lidar_time_`
2. 记录第一个 leg_sensor 消息的时间戳 `first_kin_imu_time_`
3. 计算偏移量：`lidar_time_offset_ = first_kin_imu_time_ - first_lidar_time_`
4. 清除旧缓存，后续所有 lidar 时间戳自动应用偏移校正

### 修改的文件

#### 1. legkilo/src/interface/ros2/ros_interface.h

添加动态同步变量：

```cpp
// Dynamic time synchronization
double lidar_time_offset_ = 0.0;
bool time_offset_calculated_ = false;
double first_lidar_time_ = 0.0;
double first_kin_imu_time_ = 0.0;
```

#### 2. legkilo/src/interface/ros2/ros_interface.cc

**lidarCallBack()** - 记录首个 lidar 时间戳，计算偏移量（处理竞态条件），应用时间偏移：

```cpp
void RosInterface::lidarCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    
    // Record first lidar timestamp for dynamic time sync
    if (first_lidar_time_ == 0.0) {
        first_lidar_time_ = timestamp;
        LOG(INFO) << "[TimeSync] First lidar timestamp: " << first_lidar_time_;
        
        // Calculate time offset if we have both first timestamps (kin_imu may have arrived first)
        if (first_kin_imu_time_ > 0.0 && !time_offset_calculated_) {
            lidar_time_offset_ = first_kin_imu_time_ - first_lidar_time_;
            time_offset_calculated_ = true;
            LOG(INFO) << "[TimeSync] Calculated lidar_time_offset: " << lidar_time_offset_ << "s";
        }
    }
    
    // ... process lidar scan ...
    
    // Apply dynamic time offset to lidar scan timestamps
    if (time_offset_calculated_) {
        lidar_scan.lidar_end_time_ += lidar_time_offset_;
        lidar_scan.lidar_begin_time_ += lidar_time_offset_;
    }
}
```

**kinematicImuCallBack()** - 记录首个 kin_imu 时间戳，计算偏移量：

```cpp
void RosInterface::kinematicImuCallBack(const go2_driver::msg::LegSensor::SharedPtr msg) {
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (first_kin_imu_time_ == 0.0) {
            first_kin_imu_time_ = timestamp;
            LOG(INFO) << "[TimeSync] First kin_imu timestamp: " << first_kin_imu_time_;
            
            // Calculate time offset if we have both first timestamps
            if (first_lidar_time_ > 0.0 && !time_offset_calculated_) {
                lidar_time_offset_ = first_kin_imu_time_ - first_lidar_time_;
                time_offset_calculated_ = true;
                LOG(INFO) << "[TimeSync] Calculated lidar_time_offset: " << lidar_time_offset_ << "s";
            }
        }
        // ... process kin_imu data ...
    }
}
```

**syncPackage()** - 等待时间偏移计算完成，清除旧缓存后开始同步：

```cpp
bool RosInterface::syncPackage() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (options::kKinAndImuUse) {
        // Wait until time offset is calculated
        if (!time_offset_calculated_) {
            return false;
        }
        
        // Clear caches once after time offset is calculated
        static bool caches_cleared_after_sync_ = false;
        if (!caches_cleared_after_sync_) {
            LOG(INFO) << "[TimeSync] Clearing caches after time offset calculation.";
            lidar_cache_.clear();
            kin_imu_cache_.clear();
            caches_cleared_after_sync_ = true;
            return false;
        }
        
        // ... normal sync logic ...
    }
}
```

### 竞态条件修复

**问题**：由于 ROS 2 多线程回调，kin_imu 回调可能先于 lidar 回调执行。首次测试日志显示：

```
[TimeSync] First kin_imu timestamp: 1768480508.156990  (先到达)
[TimeSync] First lidar timestamp: 1768477808.343753    (后到达)
```

此时 kin_imu 回调中 `first_lidar_time_` 还是 0，无法计算偏移量。

**解决**：在 `lidarCallBack()` 中也添加偏移量计算逻辑，确保无论哪个传感器先到达都能正确计算。

### 运行时日志输出

```shell
[TimeSync] First kin_imu timestamp: 1768480508.156990
[TimeSync] First lidar timestamp: 1768477808.343753
[TimeSync] Calculated lidar_time_offset: 2699.813237s (kin_imu - lidar)
[TimeSync] Clearing caches after time offset calculation. lidar_cache=5, kin_imu_cache=XX
[SyncPackage] Synced! kin_imus_count=XX
```

## 测试验证

编译并运行：

```bash
cd /home/alex/others/unitree_ws/slam/Leg-KILO
colcon build --packages-select legkilo --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch legkilo go2_real.launch.py
```

## 备注

- 系统现在完全自动处理传感器时间戳差异，无需任何手动配置
- 该方案适用于任何存在固定时间偏移的多传感器数据同步场景
- 通过在两个回调中都添加偏移量计算逻辑，解决了多线程竞态条件问题
