#pragma once
#include <ros/package.h>

#include <memory>

#include "hy_manipulation_controllers/kinematics/kinematics_solver.h"
#include "hy_manipulation_controllers/motion_controller/motion_controller_base.h"
namespace hy_manipulation_controllers {

struct DragParam {
  float kd = 0.1f;
  float tff = 0.0f;
};

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

  bool LoadDragParamsFromJson(const std::string& file_path) {
    std::ifstream in(file_path);
    if (!in.is_open()) {
      LOG_ERROR("DragController: Failed to open JSON file: {}", file_path);
      return false;
    }

    nlohmann::json json_data;
    try {
      in >> json_data;
    } catch (const std::exception& e) {
      LOG_ERROR("DragController: Failed to parse JSON: {}", e.what());
      return false;
    }

    drag_params_.clear();
    for (const auto& item : json_data["drag_control_parameters"]) {
      int id = item.at("id").get<int>();
      DragParam param;
      param.kd = item.at("kd").get<float>();
      param.tff = item.at("tff").get<float>();
      drag_params_[id] = param;
    }

    LOG_INFO("DragController: Loaded {} drag params from {}",
             drag_params_.size(), file_path);
    return true;
  }

 private:
  std::shared_ptr<KinematicsSolver> kinematics_solver_;
  std::unordered_map<int, DragParam> drag_params_;
};

}  // namespace hy_manipulation_controllers