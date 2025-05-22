#include "hy_picking_controllers/core/arm_controller.h"

namespace hy_picking_controllers {

ArmController::ArmController(std::string _namespace,
                             nlohmann::json _arm_params) {
  nh_ = std::make_unique<ros::NodeHandle>(_namespace);
}

ArmController::~ArmController() {}

}  // namespace hy_picking_controllers
