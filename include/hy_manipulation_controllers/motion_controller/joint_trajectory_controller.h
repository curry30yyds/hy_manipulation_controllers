#pragma once

#include "hy_manipulation_controllers/motion_controller/motion_controller_base.h"

namespace hy_manipulation_controllers {

class JointTrajectoryController : public MotionControllerBase {
 public:
  JointTrajectoryController(const MotionControllerParams& _params);

  MotionControllerType GetMotionControllerType() const override {
    return MotionControllerType::MCT_JOINT_TRAJECTORY;
  }

  void SetTrajectory(const JointTrajectory& _joint_trajectory);

  ~JointTrajectoryController();

  void Start(bool _block_flag = false) override;

  void Update(
      const std::vector<JointState>& _joint_states,
      std::vector<JointControlCommand>& _joint_control_commands) override;

  void Stop(float _duration) const override;

 private:
  JointTrajectory joint_trajectory_;
};

}  // namespace hy_manipulation_controllers