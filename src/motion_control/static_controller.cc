#include "hy_manipulation_controllers/motion_controller/static_controller.h"

namespace hy_manipulation_controllers {

StaticController::StaticController(
    const std::vector<JointControlParams>& _joint_control_params)
    : MotionControllerBase(_joint_control_params), has_target_(false) {
  if (!_joint_control_params.empty()) {
    target_positions_.resize(_joint_control_params.size());
  }
  LOG_INFO("StaticController constructed.");
}
StaticController::~StaticController() {}
void StaticController::Start(bool _block_flag) {
  motion_controller_state_ = MCS_RUNNING;
  has_target_ = false;  // 强制下一次Update重新捕获位置 防止第一次不是有效
  LOG_INFO(
      "StaticController started. Will hold position on first Update call.");
}

void StaticController::Update(
    const std::vector<JointState>& _joint_states,
    std::vector<JointControlCommand>& _joint_control_commands) {
  if (motion_controller_state_ != MCS_RUNNING) {
    _joint_control_commands.clear();
    return;
  }

  if (!has_target_) {
    if (_joint_states.empty() ||
        _joint_states.size() != target_positions_.size()) {
      LOG_ERROR(
          "StaticController: Waiting for valid joint states to "
          "capture hold position...");
      _joint_control_commands.clear();
      return;
    }

    for (size_t i = 0; i < _joint_states.size(); ++i) {
      target_positions_(i) = _joint_states[i].position;
    }
    has_target_ = true;  //获取当前位置
    LOG_INFO("StaticController: Target position captured successfully.");
  }

  _joint_control_commands.resize(target_positions_.size());
  for (size_t i = 0; i < _joint_control_commands.size(); ++i) {
    _joint_control_commands[i].id = static_cast<int>(i);
    _joint_control_commands[i].target_position = target_positions_(i);
    _joint_control_commands[i].target_velocity = 0.0f;
    _joint_control_commands[i].mit_kp = 1.0f;
    _joint_control_commands[i].mit_kd = 0.1f;
    _joint_control_commands[i].mit_t_ff = 0.0f;
  }
}

void StaticController::Stop(float _duration) {
  motion_controller_state_ = MCS_STOPPED;
  LOG_INFO("StaticController stopped.");
}

}  // namespace hy_manipulation_controllers