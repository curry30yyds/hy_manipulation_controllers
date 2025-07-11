#pragma once
#include <ros/ros.h>

#include "hy_manipulation_controllers/motion_controller/motion_controller_base.h"

namespace hy_manipulation_controllers {

class JointTrajectoryController : public MotionControllerBase {
 public:
  JointTrajectoryController(
      const std::vector<JointControlParams> &_joint_control_params);

  MotionControllerType GetMotionControllerType() const override {
    return MCT_JOINT_TRAJECTORY;
  }
  // 输入密集轨迹点
  void SetTrajectory(const JointTrajectory &_joint_trajectory) override;

  ~JointTrajectoryController();

  void Start(bool _block_flag = false) override;

  void Update(
      const std::vector<JointState> &_joint_states,
      std::vector<JointControlCommand> &_joint_control_commands) override;

  // void Stop(float _duration) const override;
  void Stop(float _duration) override;

 private:
  JointTrajectory joint_trajectory_;

  size_t trajectory_index_;

  int num_joints_;

  double Kp_ = 20.0;      // 示例比例增益
  double Kd_ = 0.1;       // 示例微分增益
  double ff_gain_ = 1.0;  // 前馈增益
  double v_max = 3.0;     // 速度限幅
};

}  // namespace hy_manipulation_controllers