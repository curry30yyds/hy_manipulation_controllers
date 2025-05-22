#pragma once

#include <hy_common/geometry/geometry_all.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

namespace hy_picking_controllers {

/**
 * @brief 机械臂控制器状态枚举
 */
enum ArmControllerState {
  AC_STATE_STANDBY = 0,
  AC_STATE_RUNNING,
  AC_STATE_ERROR
};

class ArmController {
 public:
  typedef std::shared_ptr<ArmController> Ptr;

 public:
  /**
   * @brief 构造函数
   *
   * @param _namespace 机械臂命名空间（话题前缀）
   * @param _arm_params 机械臂参数
   */
  ArmController(std::string _namespace, nlohmann::json _arm_params);

  /**
   * @brief 析构函数
   */
  ~ArmController();

  /**
   * @brief 连接机械臂（开启内部线程）
   */
  void Connect();

  /**
   * @brief 断开机械臂（关闭内部线程）
   */
  void Disconnect();

  /**
   * @brief 获取机械臂状态
   *
   * @return 机械臂状态
   */
  ArmControllerState GetArmControllerState();

 public:
  /**
   * @brief 关节空间位置控制
   *
   * @param _target_joint_positions 目标关节位置
   * @param _vel_percentage 速度百分比(0.0~1.0)
   * @param _acc_duration 加速时间
   * @param _stiffness 刚度(0.0~1.0)
   * @param _block_flag 是否阻塞
   */
  void DoJointPositionControl(const Eigen::VectorXf& _target_joint_positions,
                              const float& _vel_percentage,
                              const float& _acc_duration,
                              const float& _stiffness, const bool& _block_flag);

  /**
   * @brief 关节空间轨迹控制
   *
   * @param _target_trajectory_positions 目标关节轨迹
   * @param _vel_percentage 速度百分比(0.0~1.0)
   * @param _acc_duration 加速时间
   * @param _stiffness 刚度(0.0~1.0)
   * @param _block_flag 是否阻塞
   */
  void DoJointTrajectoryControl(
      const std::vector<Eigen::VectorXf>& _target_trajectory_positions,
      const float& _vel_percentage, const float& _acc_duration,
      const float& _stiffness, const bool& _block_flag);

  /**
   * @brief 笛卡尔空间位姿控制
   *
   * @param _target_pose 目标位姿
   * @param _max_cartesian_vel 最大笛卡尔速度
   * @param _acc_duration 加速时间
   * @param _stiffness 刚度(0.0~1.0)
   * @param _block_flag 是否阻塞
   */
  void DoCartesianPoseControl(
      const hy_common::geometry::Transform3D& _target_pose,
      const float& _max_cartesian_vel, const float& _acc_duration,
      const float& _stiffness, const bool& _block_flag);

  /**
   * @brief 笛卡尔空间轨迹控制
   *
   * @param _target_trajectory_poses 目标笛卡尔轨迹
   * @param _max_cartesian_vel 最大笛卡尔速度
   * @param _acc_duration 加速时间
   * @param _stiffness 刚度(0.0~1.0)
   * @param _block_flag 是否阻塞
   */
  void DoCartesianTrajectoryControl(
      const std::vector<hy_common::geometry::Transform3D>&
          _target_trajectory_poses,
      const float& _max_cartesian_vel, const float& _acc_duration,
      const float& _stiffness, const bool& _block_flag);

  /**
   * @brief 启动拖拽模式（重力补偿，用于示教）
   *
   * @param _stiffness 刚度(0.0~1.0)
   */
  void StartDragMode(const float& _stiffness);

  /**
   * @brief 停止拖拽模式
   */
  void StopDragMode();

  /**
   * @brief 启动静止模式（机械臂静止在当前位置）
   *
   * @param _stiffness 刚度(0.0~1.0)
   */
  void StartStaticMode(const float& _stiffness);

  /**
   * @brief 停止静止模式
   */
  void StopStaticMode();

  /**
   * @brief 启动点动模式
   *
   * @param _vel_percentage 速度百分比(0.0~1.0)
   * @param _stiffness 刚度(0.0~1.0)
   */
  void StartJoggingMode(const float& _vel_percentage, const float& _stiffness);

  /**
   * @brief 设置点动关节目标
   *
   * @param _joint_target 关节目标
   */
  void SetJoggingJointTarget(const Eigen::VectorXf& _joint_target);

  /**
   * @brief 设置点动末端位姿目标
   *
   * @param _pose_target 末端位姿目标
   */
  void SetJoggingPoseTarget(
      const hy_common::geometry::Transform3D& _pose_target);

  /**
   * @brief 停止点动模式
   */
  void StopJoggingMode();

 public:
  /**
   * @brief 正运动学求解
   *
   * @param _joint_positions_in 关节位置
   * @param _end_pose_out 末端位姿
   */
  bool SolveFK(const Eigen::VectorXf& _joint_positions_in,
               hy_common::geometry::Transform3D& _end_pose_out);

  /**
   * @brief 逆运动学求解
   *
   * @param _end_pose_in 末端位姿
   * @param _joint_positions_out 关节位置
   */
  bool SolveIK(const hy_common::geometry::Transform3D& _end_pose_in,
               Eigen::VectorXf& _joint_positions_out);

  /**
   * @brief 采样末端位姿的逆解
   *
   * @param _end_position_in 末端位置
   * @param _joint_lower_limits 关节位置下限
   * @param _joint_upper_limits 关节位置上限
   * @param _end_orientation_sample_step 末端朝向的采样步长
   * @param _solutions 逆解
   */
  bool SamplePositionIKSolutions(
      const hy_common::geometry::Point3D& _end_position_in,
      const std::vector<Eigen::VectorXf>& _joint_lower_limits,
      const std::vector<Eigen::VectorXf>& _joint_upper_limits,
      const float& _end_orientation_sample_step,
      std::vector<Eigen::VectorXf>& _solutions);

 protected:
  /**
   * @brief 状态发布线程
   *
   */
  void StatePubThread();

  /**
   * @brief 控制线程
   *
   */
  void ControlThread();

 protected:
  std::unique_ptr<ros::NodeHandle> nh_;

  std::thread control_thread_;    // 控制线程
  std::thread state_pub_thread_;  // 状态发布线程

  Eigen::VectorXf current_joint_positions_;            // 当前关节位置
  Eigen::VectorXf current_joint_velocities_;           // 当前关节速度
  hy_common::geometry::Transform3D current_end_pose_;  // 当前笛卡尔位姿
};

}  // namespace hy_picking_controllers
