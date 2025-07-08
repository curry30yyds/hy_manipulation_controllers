#pragma once

#include "hy_manipulation_controllers/motion_controller/motion_controller_base.h"

namespace hy_manipulation_controllers
{

  class JointTrajectoryController : public MotionControllerBase
  {
  public:
    JointTrajectoryController(
        const std::vector<JointControlParams> &_joint_control_params);

    MotionControllerType GetMotionControllerType() const override
    {
      return MotionControllerType::MCT_JOINT_TRAJECTORY;
    }
    // 输入密集轨迹点
    void SetTrajectory(const JointTrajectory &_joint_trajectory);

    ~JointTrajectoryController();

    void Start(bool _block_flag = false) override;

    void Update(
        const std::vector<JointState> &_joint_states,
        std::vector<JointControlCommand> &_joint_control_commands) override;

    // void Stop(float _duration) const override;
    void Stop(float _duration) override;

  private:
    JointTrajectory joint_trajectory_;
  };

} // namespace hy_manipulation_controllers