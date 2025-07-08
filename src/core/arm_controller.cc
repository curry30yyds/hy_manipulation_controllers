#include "hy_manipulation_controllers/core/arm_controller.h"
#include "hy_manipulation_controllers/motion_controller/joint_trajectory_controller.h"

namespace hy_manipulation_controllers
{

  ArmController::ArmController(const std::string &_namespace,
                               const std::string &_param_folder)
  {
    nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    // 话题发布订阅
    joint_states_sub_ = nh_->subscribe("/JointStates", 10,
                                       &ArmController::JointStateCallback, this);
    joint_cmds_pub_ = nh_->advertise<hy_hardware_interface::test_dm_4dof_control>(
        "/JointCmds", 1);

    urdf_path = _param_folder + "urdf/scara.urdf";
    cam_params_json_path = _param_folder + "/config/camera_extrinsics.json";

    control_state_ = AC_STATE_STANDBY;
  }
  // 启动线程并设置状态
  void ArmController::Connect()
  {
    if (control_state_ != AC_STATE_STANDBY)
    {
      LOG_WARN("ArmController is in {} state instead of STANDBY.", control_state_);
      return;
    }
    if (!Initialize())
    {
      LOG_ERROR("KDL chain initialization failed!");
      control_state_ = AC_STATE_ERROR;
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
    LOG_INFO("[ArmController] Disconnect requested. Current state: {}", ToString(control_state_));

    if (control_state_ == AC_STATE_STANDBY || control_state_ == AC_STATE_DISABLE)
    {
      LOG_WARN("[ArmController] No need to disconnect from {} state.", ToString(control_state_));
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

    LOG_ERROR("[ArmController] Disconnect called from unexpected state: {}", ToString(control_state_));
  }
  // 获取状态
  ArmControllerState ArmController::GetArmControllerState()
  {
    return control_state_;
  }

  bool ArmController::Initialize()
  {
    // 1. KDL chain 构建
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.good())
    {
      LOG_ERROR("URDF file does not exist: {}", urdf_path);
      control_state_ = AC_STATE_ERROR;
      return false;
    }

    std::ifstream cam_file(cam_params_json_path);
    if (!cam_file.good())
    {
      LOG_ERROR("Camera extrinsic parameter file does not exist: {}", cam_params_json_path);
      control_state_ = AC_STATE_ERROR;
      return false;
    }

    LOG_INFO("URDF path: [{}]", urdf_path);

    CameraExtrinsicParams CamParams_;
    if (!CamParams_.loadFromJson(cam_params_json_path))
    {
      LOG_ERROR("Failed to load camera extrinsic parameters from file: {}", cam_params_json_path);
      return false;
    }

    try
    {
      kinematics_solver_ = std::make_shared<KinematicsSolver>(urdf_path);
    }
    catch (const std::exception &e)
    {
      LOG_ERROR("Failed to create KinematicsSolver: {}", e.what());
      return false;
    }

    if (!kinematics_solver_->LoadCameraExtrinsics(CamParams_))
    {
      LOG_ERROR("Failed to load camera extrinsics into solver.");
      return false;
    }

    Eigen::Matrix4f extrinsics_out;
    if (!kinematics_solver_->GetCameraExtrinsics(extrinsics_out))
    {
      LOG_ERROR("Failed to retrieve camera extrinsics from solver.");
      return false;
    }

    unsigned int num_joints = kinematics_solver_->GetChain().getNrOfJoints();
    if (num_joints == 0)
    {
      LOG_ERROR("KDL chain contains 0 joints. Initialization failed.");
      return false;
    }

    LOG_INFO("[ArmController] KinematicsSolver initialized with {} joints.", num_joints);
    // 2. 底层硬件初始化
    if (!joint_states_.size())
    {
      LOG_ERROR("Hardware initialization failed: could not subscribe to /JointStates topic.");
      return false;
    }
    return true;
  }

  void ArmController::ControlThread()
  {
    ros::Rate rate(200.0);
    LOG_INFO("[ArmController] Control thread has started.");
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
    LOG_INFO("[ArmController] StatePub thread has started.");
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

  void ArmController::JointStateCallback(const hy_hardware_interface::test_dm_4dof_state::ConstPtr &msg)
  {
    joint_states_.clear();

    for (const auto &m : msg->joint_states)
    {
      JointState js;
      // js.id = m.id;
      js.position = m.position;
      js.velocity = m.velocity;
      js.torque = m.torque;
      js.temperature = m.temperature;
      js.current = m.current;
      js.voltage = m.voltage;
      js.running_state = m.running_state;
      js.error_type = m.error_type;

      joint_states_.push_back(js);
    }

    LOG_INFO("[ArmController] Received {} joints.", joint_states_.size());
  }
  bool ArmController::SolveFK(const Eigen::VectorXf &_joint_positions_in,
                              hy_common::geometry::Transform3D &_end_pose_out)
  {
    if (!kinematics_solver_->SolveFK(_joint_positions_in, _end_pose_out))
    {
      LOG_ERROR("SolveFK failed!");
      control_state_ = AC_STATE_ERROR;
      return false;
    }
  }
  bool ArmController::SolveIK(const hy_common::geometry::Transform3D &_end_pose_in,
                              Eigen::VectorXf &_joint_positions_out)
  {
    if (!kinematics_solver_->SolveIK(_end_pose_in, _joint_positions_out))
    {
      LOG_ERROR("SolveFK failed!");
      control_state_ = AC_STATE_ERROR;
      return false;
    }
  }
} // namespace hy_manipulation_controllers
