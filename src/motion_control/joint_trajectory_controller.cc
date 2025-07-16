#include "hy_manipulation_controllers/motion_controller/joint_trajectory_controller.h"

#include <algorithm>
#include <chrono>
#include <thread>

namespace hy_manipulation_controllers {

JointTrajectoryController::JointTrajectoryController(
    const std::vector<JointControlParams> &_joint_control_params)
    : MotionControllerBase(_joint_control_params) {
  num_joints_ = _joint_control_params.size();
  if (num_joints_ == 0) {
    LOG_ERROR("JointTrajectoryController initialized with zero joints!");
    motion_controller_state_ = MCS_ERROR;
  }
  LOG_INFO("JointTrajectoryController constructed for {} joints.", num_joints_);
}

JointTrajectoryController::~JointTrajectoryController() {}

void JointTrajectoryController::SetTrajectory(
    const JointTrajectory &_joint_trajectory) {
  if (num_joints_ == 0) {
    LOG_ERROR("Cannot set trajectory, controller has zero joints.");
    motion_controller_state_ = MCS_ERROR;
    return;
  }

  if (!_joint_trajectory.empty() &&
      _joint_trajectory.size() % num_joints_ != 0) {
    LOG_ERROR(
        "Invalid trajectory size. Total points {} is not a multiple of number "
        "of joints {}.",
        _joint_trajectory.size(), num_joints_);
    motion_controller_state_ = MCS_ERROR;
    return;
  }
  joint_trajectory_ = _joint_trajectory;
  trajectory_index_ = 0;
  motion_controller_state_ = MCS_STOPPED;
  if (joint_trajectory_.empty()) {
    LOG_INFO("Trajectory has been cleared.");
  } else {
    LOG_INFO("New trajectory set with {} timesteps.",
             joint_trajectory_.size() / num_joints_);
  }
}

void JointTrajectoryController::Start(bool _block_flag) {
  if (motion_controller_state_ == MCS_RUNNING) {
    LOG_WARN("Controller is already running.");
    return;
  }
  if (joint_trajectory_.empty()) {
    LOG_WARN("Cannot start, trajectory is empty.");
    return;
  }

  trajectory_index_ = 0;
  motion_controller_state_ = MCS_RUNNING;
  LOG_INFO("JointTrajectoryController started.");

  if (_block_flag) {
    ros::Rate poll_rate(100);
    while (ros::ok() && motion_controller_state_ == MCS_RUNNING) {
      poll_rate.sleep();
    }
    LOG_INFO("Blocking call finished. Controller state: {}",
             motion_controller_state_);
  }
}
void JointTrajectoryController::Update(
    const std::vector<JointState> &_joint_states,
    std::vector<JointControlCommand> &_joint_control_commands) {
  if (motion_controller_state_ != MCS_RUNNING) {
    _joint_control_commands.clear();
    return;
  }
  size_t start_idx = trajectory_index_ * num_joints_;

  // 检查是否已执行完所有时间步
  if (start_idx >= joint_trajectory_.size()) {
    LOG_INFO("Trajectory finished.");
    motion_controller_state_ = MCS_STOPPED;
    _joint_control_commands.clear();
    return;
  }

  _joint_control_commands.resize(num_joints_);

  //单独位置控制
  for (int i = 0; i < num_joints_; ++i) {
    const auto &target_point = joint_trajectory_[start_idx + i];
    _joint_control_commands[i].id = target_point.id;
    _joint_control_commands[i].target_position = target_point.position;
    _joint_control_commands[i].target_velocity = 0.0f;
    _joint_control_commands[i].mit_kp = 10.0f;
    _joint_control_commands[i].mit_kd = 0.2f;
    _joint_control_commands[i].mit_t_ff = 0.0f;
  }

  //闭环速度控制
  // for (int i = 0; i < num_joints_; ++i) {
  //   const auto &target_point = joint_trajectory_[start_idx + i];
  //   double err_pos = target_point.position - _joint_states[i].position;
  //   double err_vel = target_point.velocity - _joint_states[i].velocity;
  //   double t_ff = ff_gain_ * target_point.velocity;
  //   double v_ctrl = t_ff + err_pos * Kp_ + err_vel * Kd_;
  //   if (std::isnan(v_ctrl)) {
  //     LOG_ERROR("NaN control output detected for joint {}", i);
  //     v_ctrl = 0.0;
  //   }
  //   v_ctrl = std::max(-v_max, std::min(v_ctrl, v_max));
  //   _joint_control_commands[i].id = target_point.id;
  //   _joint_control_commands[i].target_position = target_point.position;
  //   _joint_control_commands[i].target_velocity = v_ctrl;
  //   _joint_control_commands[i].mit_kp = 15.0f;
  //   _joint_control_commands[i].mit_kd = 1.0f;
  //   _joint_control_commands[i].mit_t_ff = 0.0f;
  // }

  trajectory_index_++;
}

void JointTrajectoryController::Stop(float _duration) {
  if (motion_controller_state_ == MCS_STOPPED) {
    return;
  }

  motion_controller_state_ = MCS_STOPPED;
}

}  // namespace hy_manipulation_controllers
