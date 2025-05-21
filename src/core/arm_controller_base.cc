#include "hy_picking_controllers/core/arm_controller_base.h"

namespace hy_picking_controllers {

ArmControllerBase::ArmControllerBase(std::string _namespace,
                                     nlohmann::json _arm_params) {
  nh_ = std::make_unique<ros::NodeHandle>(_namespace);
}

ArmControllerBase::~ArmControllerBase() {}

}  // namespace hy_picking_controllers
