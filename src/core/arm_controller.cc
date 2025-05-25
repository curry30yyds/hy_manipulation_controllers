#include "hy_manipulation_controllers/core/arm_controller.h"

namespace hy_manipulation_controllers {

ArmController::ArmController(std::string _namespace,
                             nlohmann::json _arm_params) {
  nh_ = std::make_unique<ros::NodeHandle>(_namespace);
}

void ArmController::ControlThread() {
  float update_rate = 200.0;

  while (ros::ok()) {
    UpdateArmStateMsgs();

    motion_controller_current_->Update(joint_states_, joint_control_commands_);

    PublishArmControlMsgs();

    std::this_thread::sleep_for(
        std::chrono::microseconds(int64_t(1000000 / update_rate)));
  }
}

ArmController::~ArmController() {}

}  // namespace hy_manipulation_controllers
