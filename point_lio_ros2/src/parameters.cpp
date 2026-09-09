#include "parameters.h"

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;

std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
state_input state_in;
state_output state_out;
std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu = true, check_satu = true, con_frame = false, cut_frame = false;
bool use_imu_as_input = false, space_down_sample = true, publish_odometry_without_downsample = false;
int  init_map_size = 10, con_frame_num = 1;
double match_s = 81, satu_acc, satu_gyro, cut_frame_time_interval = 0.1;
float  plane_thr = 0.1f;
double filter_size_surf_min = 0.5, filter_size_map_min = 0.5, fov_deg = 180;
// double cube_len = 2000; 
float  DET_RANGE = 450;
bool   imu_en = true;
double imu_time_inte = 0.005;
double laser_point_cov = 0.01, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov; 
int    lidar_type, pcd_save_interval;
std::vector<double> gravity_init, gravity;
bool   runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
bool   scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
shared_ptr<ImuProcess> p_imu;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;
double time_diff_lidar_to_imu = 0.0;

double lidar_time_inte = 0.1, first_imu_time = 0.0;
int cut_frame_num = 1, orig_odom_freq = 10;
double online_refine_time = 20.0; //unit: s
bool cut_frame_init = false; // true;

MeasureGroup Measures;

ofstream fout_out, fout_imu_pbp;

// ROS2 参数读取：ROS1 的 nh.param<T>(key, var, def) 在 rclcpp 中声明与读取分离，
// 这里用模板把声明+读取包回一行，保持 56 个参数逐行 diff 最小
template <typename T>
T get_param(rclcpp::Node::SharedPtr &nh, const std::string &name, const T &default_value)
{
  // ROS1: nh.param<T>(key, var, def) 一步完成；ROS2 必须先声明再取，
  // 否则 get_parameter 抛 ParameterNotDeclaredException。
  // 上游 parameters.cpp 把 mapping.lidar_meas_cov 读了两次，ROS2 重复声明会抛
  // ParameterAlreadyDeclaredException，所以先 has_parameter 再声明
  if (!nh->has_parameter(name))
    nh->declare_parameter(name, default_value);
  return nh->get_parameter(name).get_value<T>();
}

void readParameters(rclcpp::Node::SharedPtr &nh)
{
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  prop_at_freq_of_imu = get_param<bool>(nh, "prop_at_freq_of_imu", 1);
  use_imu_as_input = get_param<bool>(nh, "use_imu_as_input", 0);
  check_satu = get_param<bool>(nh, "check_satu", 1);
  init_map_size = get_param<int>(nh, "init_map_size", 100);
  space_down_sample = get_param<bool>(nh, "space_down_sample", 1);
  satu_acc = get_param<double>(nh, "mapping.satu_acc", 3.0);
  satu_gyro = get_param<double>(nh, "mapping.satu_gyro", 35.0);
  acc_norm = get_param<double>(nh, "mapping.acc_norm", 1.0);
  plane_thr = get_param<float>(nh, "mapping.plane_thr", 0.05f);
  p_pre->point_filter_num = get_param<int>(nh, "point_filter_num", 2);
  lid_topic = get_param<std::string>(nh, "common.lid_topic", "/livox/lidar");
  imu_topic = get_param<std::string>(nh, "common.imu_topic", "/livox/imu");
  con_frame = get_param<bool>(nh, "common.con_frame", false);
  con_frame_num = get_param<int>(nh, "common.con_frame_num", 1);
  cut_frame = get_param<bool>(nh, "common.cut_frame", false);
  cut_frame_time_interval = get_param<double>(nh, "common.cut_frame_time_interval", 0.1);
  time_diff_lidar_to_imu = get_param<double>(nh, "common.time_diff_lidar_to_imu", 0.0);
  filter_size_surf_min = get_param<double>(nh, "filter_size_surf", 0.5);
  filter_size_map_min = get_param<double>(nh, "filter_size_map", 0.5);
  // cube_len = get_param<double>(nh, "cube_side_length", 2000);
  DET_RANGE = get_param<float>(nh, "mapping.det_range", 300.f);
  fov_deg = get_param<double>(nh, "mapping.fov_degree", 180);
  imu_en = get_param<bool>(nh, "mapping.imu_en", true);
  extrinsic_est_en = get_param<bool>(nh, "mapping.extrinsic_est_en", true);
  imu_time_inte = get_param<double>(nh, "mapping.imu_time_inte", 0.005);
  laser_point_cov = get_param<double>(nh, "mapping.lidar_meas_cov", 0.1);
  acc_cov_input = get_param<double>(nh, "mapping.acc_cov_input", 0.1);
  vel_cov = get_param<double>(nh, "mapping.vel_cov", 20);
  gyr_cov_input = get_param<double>(nh, "mapping.gyr_cov_input", 0.1);
  gyr_cov_output = get_param<double>(nh, "mapping.gyr_cov_output", 0.1);
  acc_cov_output = get_param<double>(nh, "mapping.acc_cov_output", 0.1);
  b_gyr_cov = get_param<double>(nh, "mapping.b_gyr_cov", 0.0001);
  b_acc_cov = get_param<double>(nh, "mapping.b_acc_cov", 0.0001);
  imu_meas_acc_cov = get_param<double>(nh, "mapping.imu_meas_acc_cov", 0.1);
  imu_meas_omg_cov = get_param<double>(nh, "mapping.imu_meas_omg_cov", 0.1);
  p_pre->blind = get_param<double>(nh, "preprocess.blind", 1.0);
  lidar_type = get_param<int>(nh, "preprocess.lidar_type", 1);
  p_pre->N_SCANS = get_param<int>(nh, "preprocess.scan_line", 16);
  p_pre->SCAN_RATE = get_param<int>(nh, "preprocess.scan_rate", 10);
  p_pre->time_unit = get_param<int>(nh, "preprocess.timestamp_unit", 1);
  match_s = get_param<double>(nh, "mapping.match_s", 81);
  gravity = get_param<std::vector<double>>(nh, "mapping.gravity", std::vector<double>());
  gravity_init = get_param<std::vector<double>>(nh, "mapping.gravity_init", std::vector<double>());
  extrinT = get_param<std::vector<double>>(nh, "mapping.extrinsic_T", std::vector<double>());
  extrinR = get_param<std::vector<double>>(nh, "mapping.extrinsic_R", std::vector<double>());
  publish_odometry_without_downsample = get_param<bool>(nh, "odometry.publish_odometry_without_downsample", false);
  path_en = get_param<bool>(nh, "publish.path_en", true);
  scan_pub_en = get_param<bool>(nh, "publish.scan_publish_en", 1);
  scan_body_pub_en = get_param<bool>(nh, "publish.scan_bodyframe_pub_en", 1);
  runtime_pos_log = get_param<bool>(nh, "runtime_pos_log_enable", 0);
  pcd_save_en = get_param<bool>(nh, "pcd_save.pcd_save_en", false);
  pcd_save_interval = get_param<int>(nh, "pcd_save.interval", -1);

  lidar_time_inte = get_param<double>(nh, "mapping.lidar_time_inte", 0.1);
  laser_point_cov = get_param<double>(nh, "mapping.lidar_meas_cov", 0.1);

  ivox_options_.resolution_ = get_param<float>(nh, "mapping.ivox_grid_resolution", 0.2);
  ivox_nearby_type = get_param<int>(nh, "ivox_nearby_type", 18);
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    // LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }
    p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
}

Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &rot) 
{
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<double, 3, 1> ang(x, y, z);
    return ang;
}

void open_file()
{

    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"),ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

}

void reset_cov(Eigen::Matrix<double, 24, 24> & P_init)
{
    P_init = MD(24, 24)::Identity() * 0.1;
    P_init.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6,6)::Identity() * 0.001;
}

void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
    P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
}