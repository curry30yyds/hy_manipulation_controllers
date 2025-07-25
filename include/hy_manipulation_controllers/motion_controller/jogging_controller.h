#pragma once

#include <hy_common/logger/logger.h>

#include <Eigen/Dense>
#include <mutex>

#include "hy_common/geometry/core/transform.h"
#include "hy_manipulation_controllers/motion_controller/motion_controller_base.h"
namespace hy_manipulation_controllers {

class JoggingController : public MotionControllerBase {
 public:
  JoggingController(
      const std::vector<JointControlParams>& _joint_control_params);

  ~JoggingController();

  MotionControllerType GetMotionControllerType() const override {
    return MCT_JOINT_JOGGING;
  }

  void Start(bool _block_flag = false) override;
  void Stop(float _duration) override;
  void Update(
      const std::vector<JointState>& _joint_states,
      std::vector<JointControlCommand>& _joint_control_commands) override;

  void SetTrajectory(const JointTrajectory& _joint_trajectory) override {}

  void SetTargetJoints(const Eigen::VectorXf& _target);

 private:
  Eigen::VectorXf target_joints_;
  std::mutex joints_mutex_;
  bool target_set_;
};

}  // namespace hy_manipulation_controllers
