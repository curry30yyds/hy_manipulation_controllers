#include "hy_manipulation_controllers/core/arm_controller.h"

namespace hy_manipulation_controllers {

ArmController::ArmController(const std::string &_namespace,
                             const std::string &_param_folder) {
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  // 话题发布订阅
  joint_states_sub_ = nh_->subscribe("/JointStates", 1,
                                     &ArmController::JointStateCallback, this);
  joint_cmds_pub_ = nh_->advertise<hy_hardware_interface::test_dm_4dof_control>(
      "/JointCmds", 1);

  urdf_path = _param_folder + "urdf/scara.urdf";
  cam_params_json_path = _param_folder + "/config/camera_extrinsics.json";
  joint_params_json_path = _param_folder + "/config/joint_control_params.json";

  control_state_ = AC_STATE_STANDBY;
}
// 启动线程并设置状态
void ArmController::Connect() {
  if (control_state_ != AC_STATE_STANDBY) {
    LOG_WARN("ArmController is in {} state instead of STANDBY.",
             control_state_);
    return;
  }
  if (!Initialize()) {
    LOG_ERROR("[ArmController] initialization failed!");
    control_state_ = AC_STATE_ERROR;
    return;
  }
  joint_states_.clear();
  joint_control_commands_.clear();
  latest_joint_states_.clear();
  if (!control_thread_.joinable()) {
    control_thread_ = std::thread(&ArmController::ControlThread, this);
  }

  if (!state_pub_thread_.joinable()) {
    state_pub_thread_ = std::thread(&ArmController::StatePubThread, this);
  }

  control_state_ = AC_STATE_RUNNING;

  LOG_INFO("[ArmController] Connected and threads started.");
}
// 停止线程并设置状态
void ArmController::Disconnect() {
  LOG_INFO("[ArmController] Disconnect requested. Current state: {}",
           ToString(control_state_));

  if (control_state_ == AC_STATE_STANDBY ||
      control_state_ == AC_STATE_DISABLE) {
    LOG_WARN("[ArmController] No need to disconnect from {} state.",
             ToString(control_state_));
    return;
  }

  if (control_state_ == AC_STATE_RUNNING || control_state_ == AC_STATE_ERROR) {
    LOG_INFO("[ArmController] Stopping threads...");
    if (control_thread_.joinable()) control_thread_.join();
    if (state_pub_thread_.joinable()) state_pub_thread_.join();

    control_state_ = AC_STATE_STANDBY;
    LOG_INFO("[ArmController] Disconnected. New state: STANDBY");
    return;
  }

  LOG_ERROR("[ArmController] Disconnect called from unexpected state: {}",
            ToString(control_state_));
}
// 获取状态
ArmControllerState ArmController::GetArmControllerState() {
  return control_state_;
}

bool ArmController::Initialize() {
  // 1. KDL chain 构建
  std::ifstream urdf_file(urdf_path);
  if (!urdf_file.good()) {
    LOG_ERROR("URDF file does not exist: {}", urdf_path);
    control_state_ = AC_STATE_ERROR;
    return false;
  }

  std::ifstream cam_file(cam_params_json_path);
  if (!cam_file.good()) {
    LOG_ERROR("Camera extrinsic parameter file does not exist: {}",
              cam_params_json_path);
    control_state_ = AC_STATE_ERROR;
    return false;
  }

  LOG_INFO("URDF path: [{}]", urdf_path);

  CameraExtrinsicParams CamParams_;
  if (!CamParams_.loadFromJson(cam_params_json_path)) {
    LOG_ERROR("Failed to load camera extrinsic parameters from file: {}",
              cam_params_json_path);
    return false;
  }

  try {
    kinematics_solver_ = std::make_shared<KinematicsSolver>(urdf_path);
  } catch (const std::exception &e) {
    LOG_ERROR("Failed to create KinematicsSolver: {}", e.what());
    return false;
  }

  if (!kinematics_solver_->LoadCameraExtrinsics(CamParams_)) {
    LOG_ERROR("Failed to load camera extrinsics into solver.");
    return false;
  }

  Eigen::Matrix4f extrinsics_out;
  if (!kinematics_solver_->GetCameraExtrinsics(extrinsics_out)) {
    LOG_ERROR("Failed to retrieve camera extrinsics from solver.");
    return false;
  }

  unsigned int num_joints = kinematics_solver_->GetChain().getNrOfJoints();
  if (num_joints == 0) {
    LOG_ERROR("KDL chain contains 0 joints. Initialization failed.");
    return false;
  }

  LOG_INFO("[ArmController] KinematicsSolver initialized with {} joints.",
           num_joints);
  // 2.运动器初始化
  if (!JointControlParams::loadFromJson(joint_params_json_path,
                                        joint_params_)) {
    LOG_ERROR("Failed to load joint control parameters.");
    return false;
  }

  motion_controller_current_ =
      std::make_shared<StaticController>(joint_params_);
  // 3. 底层硬件初始化
  ros::Time start_time = ros::Time::now();
  ros::Rate r(50);
  while (!joint_state_received_ &&
         (ros::Time::now() - start_time).toSec() < 1.0) {
    ros::spinOnce();
    r.sleep();
  }

  if (!joint_state_received_) {
    LOG_ERROR(
        "Hardware initialization failed: did not receive /JointStates "
        "message.");
    return false;
  } else {
    LOG_INFO("[ArmController] Received {} joints.",
             latest_joint_states_.size());
  }

  return true;
}

void ArmController::ControlThread() {
  ros::Rate rate(200.0);
  LOG_INFO("[ArmController] Control thread has started.");
  while (ros::ok()) {
    // 1. 更新机械臂状态
    UpdateArmStateMsgs();

    // 2. 更新机械臂控制命令
    {
      std::lock_guard<std::mutex> lock(motion_controller_mutex_);
      if (motion_controller_current_) {
        motion_controller_current_->Update(joint_states_,
                                           joint_control_commands_);
      }
    }

    // 3. 更新运动控制器状态
    UpdateMotionControllerState();

    // 4. 更新机械臂控制器状态
    UpdateArmControllerState();

    // 5. 发布机械臂控制命令
    PublishArmControlMsgs();

    rate.sleep();
  }
}

void ArmController::StatePubThread() {
  ros::Rate rate(200.0);
  LOG_INFO("[ArmController] StatePub thread has started.");
  while (ros::ok()) {
    UpdateArmTf();

    rate.sleep();
  }
}

ArmController::~ArmController() {}

void ArmController::PublishArmControlMsgs() {
  if (joint_control_commands_.empty()) {
    return;
  }

  hy_hardware_interface::test_dm_4dof_control cmd_msg;

  cmd_msg.joint_control_frames.resize(joint_control_commands_.size());

  for (size_t i = 0; i < joint_control_commands_.size(); ++i) {
    const auto &internal_cmd = joint_control_commands_[i];

    auto &ros_cmd_frame = cmd_msg.joint_control_frames[i];

    ros_cmd_frame.p_des = internal_cmd.target_position;
    ros_cmd_frame.v_des = internal_cmd.target_velocity;
    ros_cmd_frame.kp = internal_cmd.mit_kp;
    ros_cmd_frame.kd = internal_cmd.mit_kd;

    ros_cmd_frame.t_ff = internal_cmd.mit_t_ff;
  }

  joint_cmds_pub_.publish(cmd_msg);
}

void ArmController::UpdateArmTf() {
  //实现 TF 更新逻辑
}

void ArmController::UpdateMotionControllerState() {
  if (is_logging_) {
    std::lock_guard<std::mutex> lock(trajectory_log_mutex_);

    trajectory_log_.push_back({ros::Time::now().toSec(), joint_states_});

    // LOG_INFO("Joint 0 position:{}", joint_states_[0].position);
  }

  if (motion_controller_current_) {
    MotionControllerState current_state =
        motion_controller_current_->GetMotionControllerState();
    // 前一个状态是运行 下一个是停止 说明结束
    if (previous_motion_state_ == MCS_RUNNING && current_state == MCS_STOPPED) {
      if (is_logging_) {
        is_logging_ = false;
        LOG_INFO("Save Finnished!");

        auto traj_controller =
            std::dynamic_pointer_cast<JointTrajectoryController>(
                motion_controller_current_);
        if (traj_controller) {
          std::lock_guard<std::mutex> lock(trajectory_log_mutex_);
          auto measured_data_snapshot = trajectory_log_;

          auto planned_trajectory_snapshot =
              traj_controller->GetCurrentTrajectory();
          std::thread(&ArmController::SaveLogToFileInternal, this,
                      std::move(measured_data_snapshot),
                      std::move(planned_trajectory_snapshot))
              .detach();

          trajectory_log_.clear();
        }
      }
    }
    previous_motion_state_ = current_state;  // 更新状态，为下一次检测做准备
  }
}

void ArmController::UpdateArmControllerState() {}

void ArmController::UpdateArmStateMsgs() {
  if (!joint_state_received_) {
    LOG_ERROR("The state of joint_state_received_ is {} ",
              joint_state_received_);
    return;
  }

  std::lock_guard<std::mutex> lock(joint_state_mutex_);

  if ((ros::Time::now() - last_state_timestamp_).toSec() > 0.03) {
    // 超时 启动静止控制
    LOG_ERROR(
        "Joint state is STALE! Using last known good state from {} seconds "
        "ago.",
        (ros::Time::now() - last_state_timestamp_).toSec());
    return;
  }

  joint_states_ = latest_joint_states_;

  // 为了后续存数据
  double timestamp = last_state_timestamp_.toSec();
  joint_state_history_[timestamp] = latest_joint_states_;
}

void ArmController::JointStateCallback(
    const hy_hardware_interface::test_dm_4dof_state::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);

  latest_joint_states_.clear();
  latest_joint_states_.reserve(msg->joint_states.size());

  for (size_t i = 0; i < msg->joint_states.size(); ++i) {
    const auto &m = msg->joint_states[i];

    JointState js;
    js.id = static_cast<int>(i);
    js.position = m.position;
    js.velocity = m.velocity;
    js.torque = m.torque;
    js.temperature = m.temperature;
    js.current = m.current;
    js.voltage = m.voltage;
    js.running_state = m.running_state;
    js.error_type = m.error_type;

    latest_joint_states_.push_back(js);
  }
  joint_state_received_ = !latest_joint_states_.empty();
  last_state_timestamp_ = ros::Time::now();
  // LOG_INFO("Joint 0 vel : [{}]", joint_states_[0].velocity);
  // LOG_INFO("Joint 0 position is {}", latest_joint_states_[0].position);
}

bool ArmController::SolveFK(const Eigen::VectorXf &_joint_positions_in,
                            hy_common::geometry::Transform3D &_end_pose_out) {
  if (!kinematics_solver_->SolveFK(_joint_positions_in, _end_pose_out)) {
    return false;
  }
  return true;
}
bool ArmController::SolveIK(
    const hy_common::geometry::Transform3D &_end_pose_in,
    Eigen::VectorXf &_joint_positions_out) {
  if (!kinematics_solver_->SolveIK(_end_pose_in, _joint_positions_out)) {
    return false;
  }
  return true;
}

void ArmController::DoCartesianPoseControl(
    const hy_common::geometry::Transform3D &_target_pose,
    const float &_max_cartesian_vel, const float &_acc_duration,
    const float &_stiffness, const bool &_block_flag) {
  Eigen::VectorXf joint_positions;
  if (!SolveIK(_target_pose, joint_positions)) {
    LOG_ERROR("Failed to generate ik solution");
    control_state_ = AC_STATE_ERROR;
    return;
  }
  DoJointPositionControl(joint_positions, _max_cartesian_vel, _acc_duration,
                         _stiffness, _block_flag);
}
void ArmController::DoJointPositionControl(
    const Eigen::VectorXf &_target_joint_positions,
    const float &_vel_percentage, const float &_acc_duration,
    const float &_stiffness, const bool &_block_flag) {
  if (!motion_controller_current_) {
    LOG_ERROR("No active motion controller.");
    return;
  }

  if (_target_joint_positions.size() != joint_states_.size()) {
    LOG_ERROR("Joint size mismatch: target={}, current={}.",
              _target_joint_positions.size(), joint_states_.size());
    return;
  }

  Eigen::VectorXf current_joints(joint_states_.size());
  for (size_t i = 0; i < joint_states_.size(); ++i) {
    current_joints(i) = joint_states_[i].position;
  }

  JointTrajectory generated_trajectory;
  kinematics_solver_->InterpolateTrajectory_pose(
      current_joints, _target_joint_positions, generated_trajectory,
      _vel_percentage, _acc_duration, 200);

  {
    std::lock_guard<std::mutex> lock(trajectory_log_mutex_);
    trajectory_log_.clear();  // 清空旧日志
    is_logging_ = true;       // 打开记录开关
  }

  {
    std::lock_guard<std::mutex> lock(motion_controller_mutex_);

    motion_controller_current_ =
        std::make_shared<JointTrajectoryController>(joint_params_);

    motion_controller_current_->SetTrajectory(generated_trajectory);
    motion_controller_current_->Start();
    previous_motion_state_ =
        motion_controller_current_->GetMotionControllerState();
  }
}
void ArmController::DoCartesianTrajectoryControl(
    const std::vector<hy_common::geometry::Transform3D>
        &_target_trajectory_poses,
    const float &_max_cartesian_vel, const float &_acc_duration,
    const float &_stiffness, const bool &_block_flag) {
  current_trajectory_.clear();
  if (_target_trajectory_poses.empty()) {
    LOG_WARN(
        "DoCartesianTrajectoryControl: Received an empty target pose "
        "trajectory.");
    return;
  }
  std::vector<Eigen::VectorXf> sparse_joint_trajectory;

  Eigen::VectorXf current_joints(joint_states_.size());
  for (size_t i = 0; i < joint_states_.size(); ++i) {
    current_joints(i) = joint_states_[i].position;
  }
  sparse_joint_trajectory.push_back(current_joints);

  for (const auto &pose : _target_trajectory_poses) {
    Eigen::VectorXf joint_positions;
    if (!SolveIK(pose, joint_positions)) {
      LOG_ERROR(
          "DoCartesianTrajectoryControl: Failed to solve IK for a target pose. "
          "Aborting trajectory.");
      return;
    }
    sparse_joint_trajectory.push_back(joint_positions);
  }
  LOG_INFO("Successfully solved IK for {} sparse points.",
           sparse_joint_trajectory.size());
  DoJointTrajectoryControl(sparse_joint_trajectory, _max_cartesian_vel,
                           _acc_duration, _stiffness, _block_flag);
}
void ArmController::DoJointTrajectoryControl(
    const std::vector<Eigen::VectorXf> &_target_trajectory_positions,
    const float &_vel_percentage, const float &_acc_duration,
    const float &_stiffness, const bool &_block_flag) {
  // kinematics_solver_->InterpolateTrajectory(sparse_joint_trajectory,
  //                                           current_trajectory_, 8.0, 1600);
  kinematics_solver_->InterpolateTrajectory(
      _target_trajectory_positions, current_trajectory_, _vel_percentage,
      _acc_duration, 200);
  {
    std::lock_guard<std::mutex> lock(trajectory_log_mutex_);
    trajectory_log_.clear();  // 清空旧日志
    is_logging_ = true;       // 打开记录开关
  }

  {
    std::lock_guard<std::mutex> lock(motion_controller_mutex_);

    motion_controller_current_ =
        std::make_shared<JointTrajectoryController>(joint_params_);

    motion_controller_current_->SetTrajectory(current_trajectory_);

    motion_controller_current_->Start();
    previous_motion_state_ =
        motion_controller_current_->GetMotionControllerState();
  }

  if (_block_flag) {
    LOG_INFO("Blocking until trajectory is complete...");
    ros::Rate poll_rate(100);
    while (ros::ok() &&
           motion_controller_current_->GetMotionControllerState() ==
               MCS_RUNNING) {
      poll_rate.sleep();
    }
    LOG_INFO("Trajectory finished, unblocking.");
  }

  // TEST 输出轨迹点
  // LOG_INFO("Total number of joint trajectories: {}",
  // sparse_joint_trajectory.size()); for (size_t i = 0; i <
  // sparse_joint_trajectory.size(); ++i)
  // {
  //   LOG_INFO("Trajectory {}: ", i);
  //   for (int j = 0; j < sparse_joint_trajectory[i].size(); ++j)
  //   {
  //     LOG_INFO("  Joint {}: Position = {}", j,
  //     sparse_joint_trajectory[i](j));
  //   }
  // }
}

void ArmController::StartStaticMode(const float &_stiffness) {
  LOG_INFO("Request to start Static Mode.");

  std::lock_guard<std::mutex> lock(motion_controller_mutex_);

  motion_controller_current_ =
      std::make_shared<StaticController>(joint_params_);
  motion_controller_current_->Start();
}

void ArmController::StopStaticMode() {
  LOG_INFO(
      "Stopping Static Mode is equivalent to starting it again to hold "
      "position.");

  StartStaticMode(1.0f);
}

void ArmController::StartDragMode(const float &_stiffness) {
  LOG_INFO("Request to start Drag Mode.");

  std::lock_guard<std::mutex> lock(motion_controller_mutex_);

  motion_controller_current_ =
      std::make_shared<DragController>(joint_params_, kinematics_solver_);
  motion_controller_current_->Start();
}

void ArmController::StopDragMode() {
  LOG_INFO("Request to stop Drag Mode. Arm will hold its current position.");

  StartStaticMode(1.0f);
}

// 点动模式 还没有实现
void ArmController::StartJoggingMode(const float &_vel_percentage,
                                     const float &_stiffness) {}

void ArmController::SetJoggingJointTarget(
    const Eigen::VectorXf &_joint_target) {}

void ArmController::SetJoggingPoseTarget(
    const hy_common::geometry::Transform3D &_pose_target) {}

void ArmController::StopJoggingMode() {}

void ArmController::SaveLogToFileInternal(
    std::vector<RecordedDataPoint> measured_log,
    JointTrajectory planned_trajectory) {
  if (planned_trajectory.empty() || measured_log.empty()) {
    LOG_WARN(
        "SaveLogToFileInternal: Received empty trajectory or log data. No file "
        "will be saved.");
    return;
  }

  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "trajectory_"
     << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << ".csv";
  std::string filename = ss.str();

  std::ofstream file_stream(filename);
  if (!file_stream.is_open()) {
    LOG_ERROR("SaveLogToFileInternal: Failed to open file for writing: {}",
              filename);
    return;
  }

  file_stream << std::fixed << std::setprecision(6);
  const size_t num_joints = measured_log.front().measured_states.size();

  file_stream << "timestamp";
  for (size_t i = 0; i < num_joints; ++i) {
    file_stream << ",j" << i << "_pos_target"
                << ",j" << i << "_vel_target"
                << ",j" << i << "_acc_target"
                << ",j" << i << "_pos_measured"
                << ",j" << i << "_vel_measured";
  }
  file_stream << "\n";

  size_t num_timesteps_planned = planned_trajectory.size() / num_joints;
  size_t num_timesteps_measured = measured_log.size();
  size_t num_timesteps_to_write =
      std::min(num_timesteps_planned, num_timesteps_measured);

  if (num_timesteps_planned != num_timesteps_measured) {
    LOG_WARN(
        "Planned trajectory has {} steps, but measured log has {} steps. "
        "Writing the minimum of the two.",
        num_timesteps_planned, num_timesteps_measured);
  }

  for (size_t i = 0; i < num_timesteps_to_write; ++i) {
    // 使用规划轨迹的时间戳作为基准
    file_stream << planned_trajectory[i * num_joints].timestamp;

    const auto &measured_point = measured_log[i];

    // 遍历每一个关节
    for (size_t j = 0; j < num_joints; ++j) {
      const auto &pt = planned_trajectory[i * num_joints + j];
      const auto &measured = measured_point.measured_states[j];
      if (j == 0) {
        file_stream << "," << pt.position << "," << pt.velocity << ","
                    << pt.acceleration << "," << measured.position << ","
                    << measured.velocity;  //* 2 * M_PI / 0.072
      } else {
        file_stream << "," << pt.position << "," << pt.velocity << ","
                    << pt.acceleration << "," << measured.position << ","
                    << measured.velocity;
      }
    }
    file_stream << "\n";
  }

  file_stream.close();
  LOG_INFO("Background save task complete. Trajectory log saved to: {}",
           filename);
}

}  // namespace hy_manipulation_controllers
