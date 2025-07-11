#include <hy_common/logger/logger.h>

#include "hy_manipulation_controllers/motion_controller/motion_controller_base.h"

namespace hy_manipulation_controllers {

MotionControllerBase::MotionControllerBase(
    const std::vector<JointControlParams> &_joint_control_params) {
  joint_control_params_ = _joint_control_params;
  motion_controller_state_ = MCS_STOPPED;
  // LOG_INFO("MotionControllerBase constructed.");
}

MotionControllerBase::~MotionControllerBase() {}

}  // namespace hy_manipulation_controllers
