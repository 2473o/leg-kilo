#include "preprocess.h"
#include <pcl/common/common.h>
#include "parameters.h"

void Preprocess::robosense_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
                                     int i_sub_cloud, int num_sub_cloud, double & start_time, double & end_time)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  bool has_ring = false;
  bool has_timestamp = false;
  for (const auto& field : msg->fields) {
    if (field.name == "ring") has_ring = true;
    if (field.name == "timestamp") has_timestamp = true;
  }

  double det_range = DET_RANGE;

  if (has_ring && has_timestamp)
  {
    pcl::PointCloud<robosense_ros::Point> pl_orig;
    try {
        pcl::fromROSMsg(*msg, pl_orig);
    } catch (const std::exception& e) {
        // Fallback or print error
        // If conversion fails, maybe fields are present but structure is different?
        // But we checked fields. 
        // We will just return to avoid crash.
        // Or we could let it fall through to else block?
        // But if fields exist, else block might fail too or produce wrong result.
        // Let's print error.
        std::cerr << "[Preprocess] Error converting to robosense_ros::Point: " << e.what() << std::endl;
        return;
    }
    int plsize = pl_orig.points.size();

    if (feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize / N_SCANS + 1);
      }
    }

    int points_per_sub = plsize / num_sub_cloud;
    int start_idx = i_sub_cloud * points_per_sub;
    int end_idx = (i_sub_cloud + 1) * points_per_sub;
    if (i_sub_cloud == num_sub_cloud - 1) end_idx = plsize;

    if (start_idx >= plsize) return;

    // Fix: Ensure we use the correct timestamp even if start_idx/end_idx are slightly off or if we iterate.
    // However, original logic uses points[start_idx] and points[end_idx-1].
    start_time = pl_orig.points[start_idx].timestamp;
    end_time = pl_orig.points[end_idx - 1].timestamp;

    for (int i = start_idx; i < end_idx; i++)
    {
      if (i % point_filter_num != 0) continue;

      const auto& pt = pl_orig.points[i];
      
      // Robustness check
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      
      double range_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (range_sq < blind * blind || range_sq > det_range * det_range) continue;

      PointType added_pt;
      added_pt.x = pt.x;
      added_pt.y = pt.y;
      added_pt.z = pt.z;
      added_pt.intensity = pt.intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = (pt.timestamp - start_time) * time_unit_scale;

      if (feature_enabled)
      {
        int layer = pt.ring;
        if (layer < N_SCANS)
        {
          pl_buff[layer].push_back(added_pt);
        }
      }
      else
      {
        pl_surf.push_back(added_pt);
      }
    }
    
    if (feature_enabled)
    {
      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) continue;

        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;

        for (int i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          double vx = pl[i].x - pl[i + 1].x;
          double vy = pl[i].y - pl[i + 1].y;
          double vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl, types);
      }
    }
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();

    double msg_time = rclcpp::Time(msg->header.stamp).seconds();
    start_time = msg_time;
    end_time = msg_time;

    if (feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize / N_SCANS + 1);
      }
      
      int width = msg->width;
      int height = msg->height;
      
      for (int i = 0; i < plsize; i++)
      {
        if (i % point_filter_num != 0) continue;
        const auto& pt = pl_orig.points[i];
        double range_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (range_sq < blind * blind || range_sq > det_range * det_range) continue;

        PointType added_pt;
        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.intensity = pt.intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = 0.0;

        int layer = 0;
        if (height > 1) layer = i / width;
        else if (plsize > 0) layer = (i / (plsize / N_SCANS)) % N_SCANS;

        if (layer < N_SCANS) pl_buff[layer].push_back(added_pt);
      }

      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) continue;

        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;

        for (int i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          double vx = pl[i].x - pl[i + 1].x;
          double vy = pl[i].y - pl[i + 1].y;
          double vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl, types);
      }
    }
    else
    {
      for (int i = 0; i < plsize; i++)
      {
        if (i % point_filter_num != 0) continue;
        const auto& pt = pl_orig.points[i];
        double range_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (range_sq < blind * blind || range_sq > det_range * det_range) continue;

        PointType added_pt;
        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.intensity = pt.intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = 0.0;
        pl_surf.push_back(added_pt);
      }
    }
  }
}




/**
 * robosense_handler
 * 
 * Code From https://github.com/RuanJY/robosense_fast_lio
 * source function: robosenseM1_handler
 * ros2 branch : https://github.com/KiiiLin/fast_lio_robosenseAiry
 * Thanks to RuanJY and KiiiLin
 */
/*
void Preprocess::robosense_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
                                     int i_sub_cloud, int num_sub_cloud, double & start_time, double & end_time)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    
    // 检查点云消息是否包含 ring 和 timestamp 字段
    bool has_ring = false;
    bool has_timestamp = false;
    for (const auto& field : msg->fields) {
        if (field.name == "ring") has_ring = true;
        if (field.name == "timestamp") has_timestamp = true;
    }
    
    int plsize = 0;
    double msg_time = rclcpp::Time(msg->header.stamp).seconds();
    
    // 如果点云没有 ring 和 timestamp 字段，使用标准 PointXYZI 处理
    if (!has_ring || !has_timestamp) {
        pcl::PointCloud<pcl::PointXYZI> pl_orig;
        try {
            pcl::fromROSMsg(*msg, pl_orig);
            plsize = pl_orig.size();
            if (plsize == 0) return;
            
            // 使用消息头时间戳作为所有点的时间戳
            start_time = msg_time;
            end_time = msg_time;
            
            if (feature_enabled) {
                // 特征提取模式：需要按行组织点云
                for (int i = 0; i < N_SCANS; i++) {
                    pl_buff[i].clear();
                    pl_buff[i].reserve(plsize);
                }
                
                // 根据高度计算 ring（假设点云按行组织）
                int points_per_scan = (msg->height > 0) ? (plsize / msg->height) : (plsize / N_SCANS);
                if (points_per_scan == 0) points_per_scan = 1;
                
                for (int i = 0; i < plsize; i++) {
                    if (i % point_filter_num != 0) continue;
                    
                    double range = pl_orig.points[i].x * pl_orig.points[i].x + 
                                   pl_orig.points[i].y * pl_orig.points[i].y + 
                                   pl_orig.points[i].z * pl_orig.points[i].z;
                    if (sqrt(range) < 150 && sqrt(range) > blind) {
                        PointType added_pt;
                        added_pt.x = pl_orig.points[i].x;
                        added_pt.y = pl_orig.points[i].y;
                        added_pt.z = pl_orig.points[i].z;
                        added_pt.intensity = pl_orig.points[i].intensity;
                        added_pt.normal_x = 0;
                        added_pt.normal_y = 0;
                        added_pt.normal_z = 0;
                        added_pt.curvature = 0.0; // 没有时间戳，设为0
                        
                        // 根据索引估算 ring
                        int estimated_ring = (points_per_scan > 0) ? ((i / points_per_scan) % N_SCANS) : (i % N_SCANS);
                        if (estimated_ring < N_SCANS) {
                            pl_buff[estimated_ring].push_back(added_pt);
                        }
                    }
                }
                
                for (int j = 0; j < N_SCANS; j++) {
                    PointCloudXYZI &pl = pl_buff[j];
                    int linesize = pl.size();
                    if (linesize < 2) continue;
                    vector<orgtype> &types = typess[j];
                    types.clear();
                    types.resize(linesize);
                    linesize--;
                    for (uint i = 0; i < linesize; i++) {
                        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                        vx = pl[i].x - pl[i + 1].x;
                        vy = pl[i].y - pl[i + 1].y;
                        vz = pl[i].z - pl[i + 1].z;
                        types[i].dista = vx * vx + vy * vy + vz * vz;
                    }
                    types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
                    give_feature(pl, types);
                }
            } else {
                // 非特征提取模式
                for (int i = 0; i < plsize; i++) {
                    if (i % point_filter_num != 0) continue;
                    
                    double range = sqrt(pl_orig.points[i].x * pl_orig.points[i].x + 
                                       pl_orig.points[i].y * pl_orig.points[i].y + 
                                       pl_orig.points[i].z * pl_orig.points[i].z);
                    if (range < 150 && range > blind) {
                        PointType added_pt;
                        added_pt.x = pl_orig.points[i].x;
                        added_pt.y = pl_orig.points[i].y;
                        added_pt.z = pl_orig.points[i].z;
                        added_pt.intensity = pl_orig.points[i].intensity;
                        added_pt.normal_x = 0;
                        added_pt.normal_y = 0;
                        added_pt.normal_z = 0;
                        added_pt.curvature = 0.0; // 没有时间戳，设为0
                        pl_surf.points.push_back(added_pt);
                    }
                }
                std::cout << " point_size_downsample: " << pl_surf.size() << std::endl;
            }
            return;
        } catch (const std::exception& e) {
            std::cerr << "Error converting point cloud: " << e.what() << std::endl;
            return;
        }
    }
    
    // 如果有 ring 和 timestamp 字段，使用原始处理方式
    pcl::PointCloud<robosense_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    plsize = pl_orig.size();
    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    if (feature_enabled)
    {
        for (int i = 0; i < N_SCANS; i++)
        {
            pl_buff[i].clear();
            pl_buff[i].reserve(plsize);//too large?
        }
        int num_point_each_sub_cloud = plsize/pl_orig.width/num_sub_cloud;
        robosense_ros::Point first_point = pl_orig.points[num_point_each_sub_cloud * i_sub_cloud];
        for(int i_ori_width = 0; i_ori_width < pl_orig.width; i_ori_width ++){
            for(int i_ori_height = num_point_each_sub_cloud * i_sub_cloud;
                    i_ori_height < num_point_each_sub_cloud * (i_sub_cloud+1); i_ori_height ++) {

                robosense_ros::Point & ori_point = pl_orig.at(i_ori_width, i_ori_height);
                if(i_ori_height == num_point_each_sub_cloud * i_sub_cloud){//record time of the first point
                    start_time = ori_point.timestamp;
                }else if(i_ori_height == num_point_each_sub_cloud * (i_sub_cloud+1) - 1){//record time of the last point
                    end_time = ori_point.timestamp;
                }
                if (i_ori_height % point_filter_num != 0) {continue;}

                double range = ori_point.x * ori_point.x + ori_point.y * ori_point.y + ori_point.z * ori_point.z;
                if(sqrt(range) < 150 && sqrt(range) > blind){

                    Eigen::Vector3d pt_vec;
                    PointType added_pt;
                    added_pt.x = ori_point.x;
                    added_pt.y = ori_point.y;
                    added_pt.z = ori_point.z;
                    added_pt.intensity = ori_point.intensity;
                    added_pt.normal_x = 0;
                    added_pt.normal_y = 0;
                    added_pt.normal_z = 0;
                    added_pt.curvature = (ori_point.timestamp-start_time) * time_unit_scale; // curvature unit: ms  time_unit_scale
                    if(i_ori_width < N_SCANS){
                        pl_buff[i_ori_width].push_back(added_pt);
                    }
                }
            }
        }
        for (int j = 0; j < N_SCANS; j++)
        {
            PointCloudXYZI &pl = pl_buff[j];
            int linesize = pl.size();
            vector<orgtype> &types = typess[j];
            types.clear();
            types.resize(linesize);
            linesize--;
            for (uint i = 0; i < linesize; i++)
            {
                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
            give_feature(pl, types);
        }
    }
    else
    {
        double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
        std::vector<double> time_stamp_of_points;

        //reordered
        int num_point_each_sub_cloud = plsize/pl_orig.width/num_sub_cloud;
        robosense_ros::Point first_point = pl_orig.points[num_point_each_sub_cloud * i_sub_cloud];
        for(int i_ori_width = 0; i_ori_width < pl_orig.width; i_ori_width ++){
            for(int i_ori_height = num_point_each_sub_cloud * i_sub_cloud;
                    i_ori_height < num_point_each_sub_cloud * (i_sub_cloud+1); i_ori_height ++) {

                robosense_ros::Point & ori_point = pl_orig.at(i_ori_width, i_ori_height);
                if(i_ori_height == num_point_each_sub_cloud * i_sub_cloud){//record time of the first point
                    start_time = ori_point.timestamp;
                }else if(i_ori_height == num_point_each_sub_cloud * (i_sub_cloud+1) - 1){//record time of the last point
                    end_time = ori_point.timestamp;
                }
                if (i_ori_height % point_filter_num != 0) {continue;}

                double range = sqrt(ori_point.x * ori_point.x + ori_point.y * ori_point.y + ori_point.z * ori_point.z);
                if(range < 150 && range > blind){

                    Eigen::Vector3d pt_vec;
                    PointType added_pt;
                    added_pt.x = ori_point.x;
                    added_pt.y = ori_point.y;
                    added_pt.z = ori_point.z;
                    added_pt.intensity = ori_point.intensity;
                    added_pt.normal_x = 0;
                    added_pt.normal_y = 0;
                    added_pt.normal_z = 0;
                    added_pt.curvature = (ori_point.timestamp-start_time) * time_unit_scale; // curvature unit: ms  time_unit_scale
                    time_stamp_of_points.push_back(added_pt.curvature);
                    pl_surf.points.push_back(added_pt);
                }

            }
        }
        std::sort(time_stamp_of_points.begin(), time_stamp_of_points.end());
        std::cout << " point_size_downsample: "<< pl_surf.size()<<std::endl;
    }
}
*/
