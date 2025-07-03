#include "hy_manipulation_controllers/core/arm_controller.h"
#include "hy_manipulation_controllers/motion_controller/joint_trajectory_controller.h"
namespace hy_manipulation_controllers
{

  ArmController::ArmController(const std::string &_namespace,
                               const std::string &_param_folder)
  {
    nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    // 话题发布订阅

    // 读取参数 读取失败 状态更改 AC_STATE_ERROR

    // 初始化 失败 AC_STATE_ERROR

    // 顺利则更改状态
    control_state_ = AC_STATE_STANDBY;

    LOG_INFO("Initialized successfully!");
  }
  // 启动线程并设置状态
  void ArmController::Connect()
  {
    if (control_state_ != AC_STATE_STANDBY)
    {
      LOG_WARN("ArmController is in {} state instead of STANDBY.", control_state_);
      return;
    }

    joint_states_.clear();
    joint_control_commands_.clear();

    if (!control_thread_.joinable())
    {
      control_thread_ = std::thread(&ArmController::ControlThread, this);
    }

    if (!state_pub_thread_.joinable())
    {
      state_pub_thread_ = std::thread(&ArmController::StatePubThread, this);
    }

    control_state_ = AC_STATE_RUNNING;

    LOG_INFO("[ArmController] Connected and threads started.");
  }
  // 停止线程并设置状态
  void ArmController::Disconnect()
  {
    LOG_INFO("[ArmController] Disconnect requested. Current state: {}", control_state_);

    if (control_state_ == AC_STATE_STANDBY || control_state_ == AC_STATE_DISABLE)
    {
      LOG_WARN("[ArmController] No need to disconnect from {} state.", control_state_);
      return;
    }

    if (control_state_ == AC_STATE_RUNNING || control_state_ == AC_STATE_ERROR)
    {
      LOG_INFO("[ArmController] Stopping threads...");
      if (control_thread_.joinable())
        control_thread_.join();
      if (state_pub_thread_.joinable())
        state_pub_thread_.join();

      control_state_ = AC_STATE_STANDBY;
      LOG_INFO("[ArmController] Disconnected. New state: STANDBY");
      return;
    }

    LOG_ERROR("[ArmController] Disconnect called from unexpected state: {}", control_state_);
  }
  // 获取状态
  ArmControllerState ArmController::GetArmControllerState()
  {
    return control_state_;
  }

  void ArmController::ControlThread()
  {
    ros::Rate rate(200.0);

    while (ros::ok())
    {
      // 1. 更新机械臂状态
      UpdateArmStateMsgs();

      // 2. 更新机械臂控制命令
      motion_controller_current_->Update(joint_states_, joint_control_commands_);

      // 3. 更新运动控制器状态
      UpdateMotionControllerState();

      // 4. 更新机械臂控制器状态
      UpdateArmControllerState();

      // 5. 发布机械臂控制命令
      PublishArmControlMsgs();

      rate.sleep();
    }
  }

  void ArmController::StatePubThread()
  {
    ros::Rate rate(200.0);

    while (ros::ok())
    {
      UpdateArmTf();

      rate.sleep();
    }
  }

  ArmController::~ArmController() {}
  void ArmController::PublishArmControlMsgs()
  {
    // TODO: 实现消息发布逻辑
  }

  void ArmController::UpdateArmTf()
  {
    // TODO: 实现 TF 更新逻辑
  }

  void ArmController::UpdateMotionControllerState()
  {
    // TODO: 实现运动控制状态更新逻辑
  }

  void ArmController::UpdateArmControllerState()
  {
    // TODO: 实现控制状态更新逻辑
  }

  void ArmController::UpdateArmStateMsgs()
  {
    // TODO: 实现状态消息构建逻辑
  }
} // namespace hy_manipulation_controllers
