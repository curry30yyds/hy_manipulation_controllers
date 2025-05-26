#include "hy_manipulation_controllers/core/arm_controller.h"

namespace hy_manipulation_controllers {

ArmController::ArmController(const std::string& _namespace,
                             const std::string& _param_folder) {
  nh_ = std::make_unique<ros::NodeHandle>(_namespace);
}

void ArmController::ControlThread() {
  float update_rate = 200.0;

  while (ros::ok()) {
    // 1. 更新机械臂状态
    UpdateArmStateMsgs();

    // 2. 更新机械臂控制命令
    motion_controller_current_->Update(joint_states_, joint_control_commands_);

    // 3. 更新运动控制器状态
    UpdateMotionControllerState();

    // 4. 更新机械臂控制器状态
    UpdateArmControllerState();

    // 5. 发布机械臂控制命令
    PublishArmControlMsgs();

    std::this_thread::sleep_for(
        std::chrono::microseconds(int64_t(1000000 / update_rate)));
  }
}

void ArmController::StatePubThread() {
  ros::Rate rate(200.0);

  while (ros::ok()) {
    UpdateArmTf();

    rate.sleep();
  }
}

ArmController::~ArmController() {}

}  // namespace hy_manipulation_controllers
