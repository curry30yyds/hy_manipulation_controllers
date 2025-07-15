#include "hy_manipulation_controllers/motion_controller/drag_controller.h"

namespace hy_manipulation_controllers {

DragController::DragController(
    const std::vector<JointControlParams>& _joint_control_params,
    std::shared_ptr<KinematicsSolver> _kinematics_solver)
    : MotionControllerBase(_joint_control_params),
      kinematics_solver_(_kinematics_solver) {
  if (!kinematics_solver_) {
    LOG_ERROR("DragController FATAL: Created with a null kinematics solver!");
    motion_controller_state_ = MCS_ERROR;
  }
  std::string package_path =
      ros::package::getPath("hy_manipulation_controllers");
  std::string param_json_path_ =
      package_path + "/params/config/drag_control_parameters.json";
  if (!LoadDragParamsFromJson(param_json_path_)) {
    LOG_ERROR("Could not open the drag params!");
    motion_controller_state_ = MCS_ERROR;
  }
  LOG_INFO("DragController constructed.");
}
DragController::~DragController() {}

void DragController::Start(bool _block_flag) {
  motion_controller_state_ = MCS_RUNNING;
  LOG_INFO("DragController started. Gravity compensation is now active.");
}

void DragController::Update(
    const std::vector<JointState>& _joint_states,
    std::vector<JointControlCommand>& _joint_control_commands) {
  if (motion_controller_state_ != MCS_RUNNING) {
    _joint_control_commands.clear();
    return;
  }

  if (!kinematics_solver_ || _joint_states.empty()) {
    _joint_control_commands.clear();
    LOG_ERROR("The Drag mode start failled!");
    return;
  }

  // // 将当前关节状态转换为 Eigen 向量
  // Eigen::VectorXf current_positions(_joint_states.size());
  // for (size_t i = 0; i < _joint_states.size(); ++i) {
  //   current_positions(i) = _joint_states[i].position;
  // }

  // // 调用动力学求解器计算用于补偿重力的力矩
  // Eigen::VectorXf gravity_torques;
  // if (!kinematics_solver_->SolveGravity(current_positions, gravity_torques))
  // {
  //   LOG_ERROR(
  //       "DragController: Failed to calculate gravity torques. "
  //       "Skipping command.");
  //   _joint_control_commands.clear();
  //   return;
  // }

  // 力矩去抵消自身的重力
  _joint_control_commands.resize(_joint_states.size());
  for (size_t i = 0; i < _joint_control_commands.size(); ++i) {
    int joint_id = static_cast<int>(i);
    _joint_control_commands[i].id = joint_id;
    _joint_control_commands[i].target_position = 0.0f;
    _joint_control_commands[i].target_velocity = 0.0f;
    _joint_control_commands[i].mit_kp = 0.0f;

    auto it = drag_params_.find(joint_id);
    if (it != drag_params_.end()) {
      _joint_control_commands[i].mit_kd = it->second.kd;
      _joint_control_commands[i].mit_t_ff = it->second.tff;
    } else {
      _joint_control_commands[i].mit_kd = 0.1f;
      _joint_control_commands[i].mit_t_ff = 0.0f;
      LOG_WARN(
          "DragController: No drag param found for joint {}. Using default "
          "kd=0.1, tff=0.",
          joint_id);
    }
  }
}

void DragController::Stop(float _duration) {
  motion_controller_state_ = MCS_STOPPED;
  LOG_INFO("DragController stopped.");
}

}  // namespace hy_manipulation_controllers
