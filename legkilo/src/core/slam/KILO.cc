#include "core/slam/KILO.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <thread>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <omp.h>
#include "common/math_utils.hpp"


#include "common/glog_utils.hpp"
#include "common/timer_utils.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/eskf.h"
#include "core/slam/voxel_map.h"
#include "preprocess/state_initial.hpp"

namespace legkilo {

namespace {
inline bool time_list(PointType& x, PointType& y) { return (x.curvature < y.curvature); }

// 修改说明：以下两个匿名命名空间辅助函数当前未被任何流程调用，会触发 -Wunused-function。
// 删除的代码仅注释保留，避免直接移除历史实现；同时一并规避未开启 OpenMP 时的 pragma 警告。
// std::vector<Eigen::Vector3d> toEigenPoints(const PointCloudType& cloud) {
//     std::vector<Eigen::Vector3d> points;
//     points.reserve(cloud.points.size());
//     for (const auto& pt : cloud.points) {
//         if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
//             continue;
//         }
//         points.emplace_back(pt.x, pt.y, pt.z);
//     }
//     return points;
// }
//
// void transformCloud(const PointCloudType& input, const Eigen::Isometry3d& transform, PointCloudType& output) {
//     output.clear();
//     output.points.resize(input.points.size());
//     Eigen::Matrix3d rot = transform.rotation();
//     Eigen::Vector3d trans = transform.translation();
//
//     #pragma omp parallel for schedule(static)
//     for (size_t i = 0; i < input.points.size(); ++i) {
//         const auto& pt_in = input.points[i];
//         auto& pt_out = output.points[i];
//         Eigen::Vector3d p(pt_in.x, pt_in.y, pt_in.z);
//         Eigen::Vector3d p_out = rot * p + trans;
//         pt_out.x = static_cast<float>(p_out.x());
//         pt_out.y = static_cast<float>(p_out.y());
//         pt_out.z = static_cast<float>(p_out.z());
//         pt_out.intensity = pt_in.intensity;
//         pt_out.curvature = pt_in.curvature;
//         Eigen::Vector3d n(pt_in.normal_x, pt_in.normal_y, pt_in.normal_z);
//         Eigen::Vector3d n_out = rot * n;
//         pt_out.normal_x = static_cast<float>(n_out.x());
//         pt_out.normal_y = static_cast<float>(n_out.y());
//         pt_out.normal_z = static_cast<float>(n_out.z());
//     }
// }
}  // namespace

KILO::KILO(const std::string& config_file) { initializeFromYaml(config_file); }
KILO::~KILO() = default;

void KILO::initializeFromYaml(const std::string& config_file) {
    YamlHelper yaml_helper(config_file);

    // Mode
    // 统一复用公共模式解析函数，避免此处与 RosInterface 重复维护字符串转枚举逻辑。同 interface/ros2/ros_interface.cc
    mode_ = common::parseMode(yaml_helper.get<std::string>("mode", "slam"));

    imu_mode_only_ = yaml_helper.get<bool>("only_imu_use", true);

    // ESKF
    ESKF::Config eskf_config;
    eskf_config.vel_process_cov = yaml_helper.get<double>("vel_process_cov");
    eskf_config.imu_acc_process_cov = yaml_helper.get<double>("imu_acc_process_cov");
    eskf_config.imu_gyr_process_cov = yaml_helper.get<double>("imu_gyr_process_cov");
    eskf_config.acc_bias_process_cov = yaml_helper.get<double>("acc_bias_process_cov");
    eskf_config.gyr_bias_process_cov = yaml_helper.get<double>("gyr_bias_process_cov");
    eskf_config.kin_bias_process_cov = yaml_helper.get<double>("kin_bias_process_cov");
    eskf_config.contact_process_cov = yaml_helper.get<double>("contact_process_cov");
    eskf_config.imu_acc_meas_noise = yaml_helper.get<double>("imu_acc_meas_noise");
    eskf_config.imu_acc_z_meas_noise = yaml_helper.get<double>("imu_acc_z_meas_noise");
    eskf_config.imu_gyr_meas_noise = yaml_helper.get<double>("imu_gyr_meas_noise");
    eskf_config.kin_meas_noise = yaml_helper.get<double>("kin_meas_noise");
    eskf_config.chd_meas_noise = yaml_helper.get<double>("chd_meas_noise");
    eskf_config.contact_meas_noise = yaml_helper.get<double>("contact_meas_noise");
    eskf_config.lidar_point_meas_ratio = yaml_helper.get<double>("lidar_point_meas_ratio");
    eskf_ = std::make_unique<ESKF>(eskf_config);

    // Init method
    gravity_ = yaml_helper.get<double>("gravity", 9.81);
    if (imu_mode_only_) {
        state_initial_ = std::make_unique<StateInitialByImu>(gravity_);
    } else {
        state_initial_ = std::make_unique<StateInitialByKinImu>(gravity_);
    }

    // Voxel map
    VoxelMapConfig voxel_map_config;
    voxel_map_config.is_pub_plane_map_ = yaml_helper.get<bool>("pub_plane_en");
    voxel_map_config.max_layer_ = yaml_helper.get<int>("max_layer");
    voxel_map_config.max_voxel_size_ = yaml_helper.get<double>("voxel_size");
    voxel_map_config.planner_threshold_ = yaml_helper.get<double>("min_eigen_value");
    voxel_map_config.sigma_num_ = yaml_helper.get<double>("sigma_num");
    voxel_map_config.beam_err_ = yaml_helper.get<double>("beam_err");
    voxel_map_config.dept_err_ = yaml_helper.get<double>("dept_err");
    voxel_map_config.layer_init_num_ = yaml_helper.get<std::vector<int>>("layer_init_num");
    voxel_map_config.max_points_num_ = yaml_helper.get<int>("max_points_num");
    voxel_map_config.map_sliding_en = yaml_helper.get<bool>("map_sliding_en");
    voxel_map_config.half_map_size = yaml_helper.get<int>("half_map_size");
    voxel_map_config.sliding_thresh = yaml_helper.get<double>("sliding_thresh");
    
    // odom_only 以局部里程计为主，若关闭滑窗清理，体素地图会持续扩张并推高常驻内存。
    // voxel_map_config.map_sliding_en = yaml_helper.get<bool>("map_sliding_en");
    if (mode_ == common::Mode::OdomOnly && !voxel_map_config.map_sliding_en) {
        voxel_map_config.map_sliding_en = true;
        LOG(INFO) << "Force enable map sliding in odom_only mode to bound voxel map memory usage";
    }
    // 修改说明：slam 模式若配置了 half_map_size/sliding_thresh 却未开启滑窗，局部地图参数实际上不会生效，常驻内存仍会继续增长。
    if (mode_ == common::Mode::Slam && !voxel_map_config.map_sliding_en) {
        LOG(WARNING) << "Map sliding is disabled in slam mode; half_map_size=" << voxel_map_config.half_map_size
                     << " and sliding_thresh=" << voxel_map_config.sliding_thresh
                     << " will not take effect, so resident memory may keep growing";
    }
    map_manager_ = std::make_unique<VoxelMapManager>(voxel_map_config);

    // Extrinsic
    std::vector<double> ext_t = yaml_helper.get<std::vector<double>>("extrinsic_T");
    std::vector<double> ext_R = yaml_helper.get<std::vector<double>>("extrinsic_R");
    ext_rot_ << MAT_FROM_ARRAY(ext_R);
    ext_t_ << VEC_FROM_ARRAY(ext_t);
    map_manager_->extT_ = ext_t_;
    map_manager_->extR_ = ext_rot_;

    // Downsample
    float voxel_grid_resolution = yaml_helper.get<float>("voxel_grid_resolution");
    voxel_grid_.setLeafSize(voxel_grid_resolution, voxel_grid_resolution, voxel_grid_resolution);
}

Vec3D KILO::getPos() const { return eskf_->getPos(); }
Mat3D KILO::getRot() const { return eskf_->getRot(); }

void KILO::cloudLidarToWorld(const CloudPtr& cloud_lidar, CloudPtr& cloud_world) {
    cloud_world->clear();
    cloud_world->points.resize(cloud_lidar->points.size());
    for (size_t i = 0; i < cloud_lidar->points.size(); ++i) {
        pointLidarToWorld(cloud_lidar->points[i], cloud_world->points[i]);
    }
}

inline void KILO::pointLidarToWorld(const PointType& point_lidar, PointType& point_world) {
    Eigen::Vector3d pt_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
    Eigen::Vector3d pt_imu = ext_rot_ * pt_lidar + ext_t_;
    Eigen::Vector3d pt_world = eskf_->getRot() * pt_imu + eskf_->getPos();

    point_world.x = static_cast<float>(pt_world(0));
    point_world.y = static_cast<float>(pt_world(1));
    point_world.z = static_cast<float>(pt_world(2));
    point_world.intensity = point_lidar.intensity;
}

bool KILO::predictUpdatePoint(double current_time, size_t idx_i, size_t idx_j, const PointCloudType& cloud_down_body,
                              PointCloudType* cloud_down_world, size_t& success_pts_size_out) {
    // 1) Predict state
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    // 2) Residuals
    size_t points_size = idx_j - idx_i;
    std::vector<PointToPlane> ptpl_list;
    std::vector<pointWithVar> pv_list(points_size);
    ptpl_list.reserve(points_size);

    std::vector<PointToPlane> ptpl_list_temp(points_size);
    std::vector<bool> is_success_list(points_size, false);

    #pragma omp parallel for schedule(static)
    for (size_t i = 0; i < points_size; ++i) {
        PointType const& cur_pt = cloud_down_body.points[i + idx_i];

        // 2.1 point var(body and world) compute
        pointWithVar& cur_pt_var = pv_list[i];
        cur_pt_var.point_b << cur_pt.x, cur_pt.y, cur_pt.z;
        cur_pt_var.point_i = ext_rot_ * cur_pt_var.point_b + ext_t_;
        cur_pt_var.point_w = eskf_->getRot() * cur_pt_var.point_i + eskf_->getPos();
        // 仅在需要对外发布点云时回填世界系点云，odom_only 下跳过这部分输出写入。
        if (cloud_down_world != nullptr) {
            cloud_down_world->points[idx_i + i].x = cur_pt_var.point_w(0);
            cloud_down_world->points[idx_i + i].y = cur_pt_var.point_w(1);
            cloud_down_world->points[idx_i + i].z = cur_pt_var.point_w(2);
            cloud_down_world->points[idx_i + i].intensity = cur_pt.intensity; // 保留u原始强度值
        }
        calcBodyCov(cur_pt_var.point_b, map_manager_->config_setting_.dept_err_,
                    map_manager_->config_setting_.beam_err_, cur_pt_var.body_var);
        cur_pt_var.point_crossmat << SKEW_SYM_MATRIX(cur_pt_var.point_i);
        Mat3D rot_extR = eskf_->getRot() * ext_rot_;
        Mat3D rot_crossmat = eskf_->getRot() * cur_pt_var.point_crossmat;
        cur_pt_var.var = rot_extR * cur_pt_var.body_var * rot_extR.transpose() +
                         rot_crossmat * eskf_->getRotCov() * rot_crossmat.transpose() + eskf_->getPosCov();

        // 2.2 residual
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = cur_pt_var.point_w[j] / map_manager_->config_setting_.max_voxel_size_;
            if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
        }
        Eigen::Vector3i position((int)loc_xyz[0], (int)loc_xyz[1], (int)loc_xyz[2]);
        auto iter = map_manager_->voxel_map_.find(position);
        if (iter != map_manager_->voxel_map_.end()) {
            VoxelOctoTree* current_octo = iter->second;
            PointToPlane single_ptpl;
            bool is_success = false;
            double prob = 0;
            map_manager_->build_single_residual(cur_pt_var, current_octo, 0, is_success, prob, single_ptpl);
            if (!is_success) {
                Eigen::Vector3i near_position = position;
                if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x() = near_position.x() + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) {
                    near_position.x() = near_position.x() - 1;
                }
                if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y() = near_position.y() + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) {
                    near_position.y() = near_position.y() - 1;
                }
                if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z() = near_position.z() + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) {
                    near_position.z() = near_position.z() - 1;
                }
                auto iter_near = map_manager_->voxel_map_.find(near_position);
                if (iter_near != map_manager_->voxel_map_.end()) {
                    map_manager_->build_single_residual(cur_pt_var, iter_near->second, 0, is_success, prob,
                                                        single_ptpl);
                }
            }
            if (is_success) {
                ptpl_list_temp[i] = single_ptpl;
                is_success_list[i] = true;
            }
        }
    }

    for (size_t i = 0; i < points_size; ++i) {
        if (is_success_list[i]) {
            ++success_pts_size_out;
            ptpl_list.push_back(ptpl_list_temp[i]);
        }
    }

    // 3) KF update with points
    size_t effect_num = ptpl_list.size();
    bool eskf_update = effect_num > 0;
    if (eskf_update) {
        ObsShared obs_shared;
        obs_shared.pt_h.resize(effect_num, 6);
        obs_shared.pt_R.resize(effect_num);
        obs_shared.pt_z.resize(effect_num);
        
        #pragma omp parallel for schedule(static)
        for (size_t k = 0; k < effect_num; ++k) {
            Vec3D crossmat_rotT_u = ptpl_list[k].point_crossmat_ * eskf_->getRot().transpose() * ptpl_list[k].normal_;
            obs_shared.pt_h.row(k) << crossmat_rotT_u(0), crossmat_rotT_u(1), crossmat_rotT_u(2),
                ptpl_list[k].normal_(0), ptpl_list[k].normal_(1), ptpl_list[k].normal_(2);

            obs_shared.pt_z(k) = -ptpl_list[k].dis_to_plane_;

            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = ptpl_list[k].point_w_ - ptpl_list[k].center_;
            J_nq.block<1, 3>(0, 3) = -ptpl_list[k].normal_;
            Mat3D var;
            var = eskf_->getRot() * ext_rot_ * ptpl_list[k].body_cov_ * ext_rot_.transpose() *
                  eskf_->getRot().transpose();
            double single_l = J_nq * ptpl_list[k].plane_var_ * J_nq.transpose();
            obs_shared.pt_R(k) = eskf_->config().lidar_point_meas_ratio *
                                 (single_l + ptpl_list[k].normal_.transpose() * var * ptpl_list[k].normal_);
        }
        eskf_->updateByPoints(obs_shared);
        last_state_update_time_ = current_time;
    }

    // 4) voxel map update
    if (eskf_update) {
        #pragma omp parallel for schedule(static)
        for (size_t i = 0; i < points_size; ++i) {
            // recompute world with updated state and update var
            pv_list[i].point_w = eskf_->getRot() * pv_list[i].point_i + eskf_->getPos();
            // 仅在需要对外发布点云时更新世界系输出缓存，避免 odom_only 下无意义写回。
            if (cloud_down_world != nullptr) {
                cloud_down_world->points[idx_i + i].x = pv_list[i].point_w(0);
                cloud_down_world->points[idx_i + i].y = pv_list[i].point_w(1);
                cloud_down_world->points[idx_i + i].z = pv_list[i].point_w(2);
            }

            Mat3D rot_extR = eskf_->getRot() * ext_rot_;
            Mat3D rot_crossmat = eskf_->getRot() * pv_list[i].point_crossmat;
            pv_list[i].var = rot_extR * pv_list[i].body_var * rot_extR.transpose() +
                             rot_crossmat * eskf_->getRotCov() * rot_crossmat.transpose() + eskf_->getPosCov();
        }
    }
    map_manager_->UpdateVoxelMap(pv_list);
    if (map_manager_->config_setting_.map_sliding_en) {
        // 地图更新后同步当前位置，并按配置触发局部滑窗清理。
        map_manager_->position_last_ = eskf_->getPos();
        if (map_manager_->needSliding()) {
            map_manager_->mapSliding();
        }
    }
    return effect_num > 0;
}

bool KILO::predictUpdateImu(const sensor_msgs::msg::Imu::SharedPtr& imu) {
    double current_time = rclcpp::Time(imu->header.stamp).seconds();
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    ObsShared obs_shared;
    obs_shared.ki_R.resize(6);
    obs_shared.ki_z.resize(6);
    Vec3D imu_acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    Vec3D imu_gyr(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R << eskf_->config().imu_acc_meas_noise, eskf_->config().imu_acc_meas_noise,
        eskf_->config().imu_acc_z_meas_noise, eskf_->config().imu_gyr_meas_noise, eskf_->config().imu_gyr_meas_noise,
        eskf_->config().imu_gyr_meas_noise;

    eskf_->updateByImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

bool KILO::predictUpdateKinImu(const common::KinImuMeas& kin_imu) {
    double current_time = kin_imu.time_stamp_;
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    int contact_nums = 0;
    for (int i = 0; i < 4; ++i) {
        if (kin_imu.contact_[i]) { contact_nums++; }
    }

    ObsShared obs_shared;
    obs_shared.ki_R.resize(6 + 3 * contact_nums);
    obs_shared.ki_z.resize(6 + 3 * contact_nums);
    obs_shared.ki_h.resize(6 + 3 * contact_nums, DIM_STATE);
    obs_shared.ki_h.setZero();

    obs_shared.ki_h.block<6, 6>(0, 9) = Eigen::Matrix<double, 6, 6>::Identity();
    obs_shared.ki_h.block<6, 6>(0, 18) = Eigen::Matrix<double, 6, 6>::Identity();
    Vec3D imu_acc(kin_imu.acc_[0], kin_imu.acc_[1], kin_imu.acc_[2]);
    Vec3D imu_gyr(kin_imu.gyr_[0], kin_imu.gyr_[1], kin_imu.gyr_[2]);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R.block<6, 1>(0, 0) << eskf_->config().imu_acc_meas_noise, eskf_->config().imu_acc_meas_noise,
        eskf_->config().imu_acc_z_meas_noise, eskf_->config().imu_gyr_meas_noise, eskf_->config().imu_gyr_meas_noise,
        eskf_->config().imu_gyr_meas_noise;

    int idx = 0;
    Mat3D w_skew = SKEW_SYM_MATRIX(eskf_->state().imu_w_);
    for (int i = 0; i < 4; ++i) {
        if (kin_imu.contact_[i]) {
            Vec3D foot_pos(kin_imu.foot_pos_[i][0], kin_imu.foot_pos_[i][1], kin_imu.foot_pos_[i][2]);
            Vec3D foot_vel(kin_imu.foot_vel_[i][0], kin_imu.foot_vel_[i][1], kin_imu.foot_vel_[i][2]);

            Vec3D w_skew_pos_vel = w_skew * foot_pos + foot_vel;

            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 0) = -eskf_->getRot() * SKEW_SYM_MATRIX(w_skew_pos_vel);
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 6) = Mat3D::Identity();
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 21) = -eskf_->getRot() * SKEW_SYM_MATRIX(foot_pos);

            obs_shared.ki_z.block<3, 1>(6 + 3 * idx, 0) = -eskf_->getVel() - eskf_->getRot() * w_skew_pos_vel;

            obs_shared.ki_R.block<3, 1>(6 + 3 * idx, 0) << eskf_->config().kin_meas_noise,
                eskf_->config().kin_meas_noise, eskf_->config().kin_meas_noise;
            idx++;
        }
    }

    eskf_->updateByKinImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

bool KILO::process(common::MeasGroup measure, CloudPtr& cloud_down_body_out, CloudPtr& cloud_down_world_out,
                   size_t& success_pts_size_out) {
    success_pts_size_out = 0;

    // odom_only 仍保留定位与建图所需内部流程，但关闭对外点云输出缓存。
    const bool need_pointcloud_output = mode_ != common::Mode::OdomOnly;

    CloudPtr cloud_raw = measure.lidar_scan_.cloud_;
    auto& imus = measure.imus_;
    auto& kin_imus = measure.kin_imus_;
    double begin_time = measure.lidar_scan_.lidar_begin_time_;
    double end_time = measure.lidar_scan_.lidar_end_time_;

    if (cloud_raw->points.empty() || (imu_mode_only_ && imus.empty()) || (!imu_mode_only_ && kin_imus.empty())) {
        LOG(WARNING) << "Data packet is not ready";
        return false;
    }

    // First-frame initialization
    if (init_flag_) {
        state_initial_->processing(measure, *eskf_);

        CloudPtr initial_cloud_world(new PointCloudType());
        this->cloudLidarToWorld(cloud_raw, initial_cloud_world);
        map_manager_->feats_down_body_ = cloud_raw;
        map_manager_->feats_down_world_ = initial_cloud_world;
        map_manager_->BuildVoxelMap(eskf_->getRot(), eskf_->getRotCov(), eskf_->getPosCov());
        if (need_pointcloud_output) {
            cloud_down_body_out = cloud_raw;
            cloud_down_world_out = initial_cloud_world;
        } else {
            // odom_only 首帧仅保留内部建图所需点云，对外输出指针保持为空。
            cloud_down_body_out.reset();
            cloud_down_world_out.reset();
        }
        
        // 初始化滑窗参考位置，避免首帧后的第一次更新立即触发整图清理。
        map_manager_->position_last_ = eskf_->getPos();
        map_manager_->last_slide_position = map_manager_->position_last_;

        auto gravity_vec = eskf_->state().grav_;
        auto bw = eskf_->state().bw_;
        LOG(INFO) << "Initialization is finished";
        LOG(INFO) << "Gravity is initialized to " << std::fixed << std::setprecision(3) << gravity_vec(0) << " "
                  << gravity_vec(1) << " " << gravity_vec(2);
        LOG(INFO) << "IMU bw is initialized to " << bw(0) << " " << bw(1) << " " << bw(2);

        init_flag_ = false;
        acc_norm_ = state_initial_->getAccNorm();
        last_state_predict_time_ = end_time;
        last_state_update_time_ = end_time;
        return true;
    }

    // Downsampling
    CloudPtr cloud_down_body(new PointCloudType());
    Timer::measure("Downsampling", [&, this]() {
        voxel_grid_.setInputCloud(cloud_raw);
        voxel_grid_.filter(*cloud_down_body);
    });

    // 按模式决定是否分配 world 输出缓存，避免 odom_only 下额外内存分配。
    PointCloudType* cloud_down_world_ptr = nullptr;
    if (need_pointcloud_output) {
        cloud_down_body_out = cloud_down_body;
        cloud_down_world_out.reset(new PointCloudType());
        cloud_down_world_out->points.resize(cloud_down_body->points.size());
        cloud_down_world_ptr = cloud_down_world_out.get();
    } else {
        // cloud_down_body_out = cloud_down_body;
        // cloud_down_world_out.reset(new PointCloudType());
        cloud_down_body_out.reset();
        cloud_down_world_out.reset();
    }

    // State predict/update & Map update
    Timer::measure("State predict/update & Map update", [&, this]() {
        // Sort by per-point time offset (curvature field)
        auto& pts = cloud_down_body->points;
        std::sort(pts.begin(), pts.end(), time_list);

        // Predict/update cycle across time-buckets of equal curvature
        const size_t pts_size = pts.size();
        size_t idx_i = 0;
        while (idx_i < pts_size) {
            double cur_point_time = begin_time + pts[idx_i].curvature;
            size_t idx_j = idx_i + 1;
            while (idx_j < pts_size && pts[idx_i].curvature == pts[idx_j].curvature) { idx_j++; }

            if (imu_mode_only_) {
                while (!imus.empty() && rclcpp::Time(imus.front()->header.stamp).seconds() < cur_point_time) {
                    this->predictUpdateImu(imus.front());
                    imus.pop_front();
                }
            } else {
                while (!kin_imus.empty() && kin_imus.front().time_stamp_ < cur_point_time) {
                    this->predictUpdateKinImu(kin_imus.front());
                    kin_imus.pop_front();
                }
            }

            this->predictUpdatePoint(cur_point_time, idx_i, idx_j, *cloud_down_body, cloud_down_world_ptr,
                                     success_pts_size_out);
            idx_i = idx_j;
        }
    });

    return true;
}

}  // namespace legkilo
