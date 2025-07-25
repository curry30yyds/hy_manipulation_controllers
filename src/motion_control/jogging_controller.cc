#include "hy_manipulation_controllers/motion_controller/jogging_controller.h"

namespace hy_manipulation_controllers {

JoggingController::JoggingController(
    const std::vector<JointControlParams>& _joint_control_params)
    : MotionControllerBase(_joint_control_params), target_set_(false) {
  LOG_INFO("JoggingController (Position Mode) constructed.");
  target_joints_.resize(_joint_control_params.size());
}

JoggingController::~JoggingController() {}

void JoggingController::Start(bool _block_flag) {
  LOG_INFO("JoggingController started.");
  motion_controller_state_ = MCS_RUNNING;
  target_set_ = false;  // 启动时无目标
}

void JoggingController::Stop(float _duration) {
  LOG_INFO("JoggingController stopped.");
  motion_controller_state_ = MCS_STOPPED;
}

void JoggingController::SetTargetJoints(const Eigen::VectorXf& _target) {
  std::lock_guard<std::mutex> lock(joints_mutex_);
  if (_target.size() == target_joints_.size()) {
    target_joints_ = _target;
    target_set_ = true;
  } else {
    LOG_ERROR("JoggingController: Target joint size mismatch!");
  }
}

void JoggingController::Update(
    const std::vector<JointState>& _joint_states,
    std::vector<JointControlCommand>& _joint_control_commands) {
  if (motion_controller_state_ != MCS_RUNNING || !target_set_) {
    _joint_control_commands.clear();
    return;
  }
  if (_joint_states.empty()) {
    _joint_control_commands.clear();
    LOG_ERROR("The joint_states is empty!");
    return;
  }
  Eigen::VectorXf current_target;
  {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    current_target = target_joints_;
  }
  if (_joint_states.size() != current_target.size()) {
    _joint_control_commands.clear();
    LOG_ERROR("The joint_states size is unequal to target_joints!");
    return;
  }

  // 位置控制
  _joint_control_commands.resize(current_target.size());
  for (size_t i = 0; i < current_target.size(); ++i) {
    auto& cmd = _joint_control_commands[i];
    const auto& params = joint_control_params_[i];
    cmd.id = i;
    cmd.target_position = current_target(i);
    cmd.target_velocity = 0;
    cmd.mit_kp = params.mit_kp;
    cmd.mit_kd = params.mit_kd;
    cmd.mit_t_ff = 0;
  }
}

}  // namespace hy_manipulation_controllers
