#pragma once
#include <memory>

#include "hy_manipulation_controllers/kinematics/kinematics_solver.h"
#include "hy_manipulation_controllers/motion_controller/motion_controller_base.h"

namespace hy_manipulation_controllers {

class DragController : public MotionControllerBase {
 public:
  DragController(const std::vector<JointControlParams>& _joint_control_params,
                 std::shared_ptr<KinematicsSolver> _kinematics_solver);
  ~DragController();

  MotionControllerType GetMotionControllerType() const override {
    return MCT_DRAG;
  }

  void Start(bool _block_flag = false) override;

  void Update(
      const std::vector<JointState>& _joint_states,
      std::vector<JointControlCommand>& _joint_control_commands) override;

  void Stop(float _duration) override;

  void SetTrajectory(const JointTrajectory& _joint_trajectory) override {}

 private:
  std::shared_ptr<KinematicsSolver> kinematics_solver_;
};

}  // namespace hy_manipulation_controllers