#pragma once

#include <memory>
#include <vector>

#include "hy_manipulation_controllers/core/arm_controller_params.h"
#include "hy_manipulation_controllers/utils/utils_common.h"

namespace hy_manipulation_controllers
{

  enum MotionControllerType
  {
    MCT_JOINT_TRAJECTORY = 0, // 关节轨迹
    MCT_JOINT_JOGGING,        // 关节点动
    MCT_STATIC,               // 静止
    MCT_DRAG,                 // 示教
  };

  enum MotionControllerState
  {
    MCS_RUNNING,  // 运行中
    MCS_STOPPING, // 减速中
    MCS_STOPPED,  // 正常停止
    MCS_ERROR,    // 错误
  };

  class MotionControllerBase
  {
  public:
    typedef std::shared_ptr<MotionControllerBase> Ptr;

  public:
    MotionControllerBase(
        const std::vector<JointControlParams> &_joint_control_params);

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
        const std::vector<JointState> &_joint_states,
        std::vector<JointControlCommand> &_joint_control_commands) = 0;

    /**
     * @brief 手动停止控制
     *
     * @param _duration
     * 减速时间(s)，若加速度无法满足，实际以各关节最长减速时间为准
     */
    // virtual void Stop(float _duration) const = 0;
    virtual void Stop(float _duration) = 0;
    /**
     * @brief 获取运动控制器状态
     *
     * @return 运动控制器状态
     */
    MotionControllerState GetMotionControllerState() const
    {
      return motion_controller_state_;
    }

    virtual void SetTrajectory(const JointTrajectory &_joint_trajectory) = 0;

  protected:
    std::vector<JointControlParams> joint_control_params_;

    MotionControllerState motion_controller_state_;
  };

} // namespace hy_manipulation_controllers
