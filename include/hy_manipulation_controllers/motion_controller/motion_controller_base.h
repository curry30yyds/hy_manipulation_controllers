#pragma once

#include <memory>
#include <vector>

#include "hy_manipulation_controllers/utils/utils_common.h"

namespace hy_manipulation_controllers {

enum MotionControllerType {
  MCT_JOINT_TRAJECTORY = 0,  // 关节轨迹
  MCT_JOINT_JOGGING,         // 关节点动
  MCT_STATIC,                // 静止
  MCT_DRAG,                  // 示教
};

enum MotionControllerState {
  MCS_RUNNING,   // 运行中
  MCS_STOPPING,  // 减速中
  MCS_STOPPED,   // 正常停止
  MCS_ERROR,     // 错误
};

class JointParams {
 public:
  int id;
  std::string type;  // "LINEAR" or "ROTATIONAL"
  float reduction_ratio;
  float max_vel;             // rad/s or m/s
  float max_acc;             // rad/s^2 or m/s^2
  std::string control_mode;  // "POSITION" or "VELOCITY" or "TORQUE" or "MIT"
};

class MotionControllerParams {
 public:
  std::vector<JointParams> joint_control_params;

  float update_rate;
};

class MotionControllerBase {
 public:
  typedef std::shared_ptr<MotionControllerBase> Ptr;

 public:
  MotionControllerBase(const MotionControllerParams& _params);
  ~MotionControllerBase();

  /**
   * @brief 获取运动控制器类型
   *
   * @return 运动控制器类型
   */
  virtual MotionControllerType GetMotionControllerType() const = 0;

  /**
   * @brief 开始控制
   *
   * @param _block_flag 是否阻塞等待
   */
  virtual void Start(bool _block_flag = false) = 0;

  /**
   * @brief 更新关节控制命令
   *
   * @param _joint_states (in) 关节状态
   * @param _joint_control_commands (out) 关节控制命令
   */
  virtual void Update(
      const std::vector<JointState>& _joint_states,
      std::vector<JointControlCommand>& _joint_control_commands) = 0;

  /**
   * @brief 手动停止控制
   *
   * @param _duration
   * 减速时间(s)，若加速度无法满足，实际以各关节最长减速时间为准
   */
  virtual void Stop(float _duration) const = 0;

  /**
   * @brief 获取运动控制器状态
   *
   * @return 运动控制器状态
   */
  MotionControllerState GetMotionControllerState() const {
    return motion_controller_state_;
  }

 protected:
  MotionControllerParams motion_controller_params_;

  MotionControllerState motion_controller_state_;
};

}  // namespace hy_manipulation_controllers
