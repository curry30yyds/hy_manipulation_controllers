#pragma once
#include <Eigen/Core>

#include "hy_manipulation_controllers/motion_controller/motion_controller_base.h"

namespace hy_manipulation_controllers {

class StaticController : public MotionControllerBase {
 public:
  StaticController(
      const std::vector<JointControlParams>& _joint_control_params);
  ~StaticController();

  MotionControllerType GetMotionControllerType() const override {
    return MCT_STATIC;
  }

  void Start(bool _block_flag = false) override;

  void Update(
      const std::vector<JointState>& _joint_states,
      std::vector<JointControlCommand>& _joint_control_commands) override;

  void Stop(float _duration) override;

  void SetTrajectory(const JointTrajectory& _joint_trajectory) override {}

 private:
  Eigen::VectorXf target_positions_;
  bool has_target_;
};

}  // namespace hy_manipulation_controllers