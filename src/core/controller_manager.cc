#include "hy_picking_controllers/core/controller_manager.h"

namespace hy_picking_controllers {

ControllerManager::ControllerManager(ros::NodeHandle& _nh,
                                     ros::NodeHandle& _nh_private)
    : nh_(_nh), nh_private_(_nh_private) {
  LOG_INFO("ControllerManager Initialized");
}

ControllerManager::~ControllerManager() {
  LOG_INFO("Controller Manager Destructed");
}

}  // namespace hy_picking_controllers