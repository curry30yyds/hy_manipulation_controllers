#pragma once

#include <hy_common/geometry/geometry_all.h>
#include <ros/ros.h>
#include <hy_common/logger/logger.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include "hy_manipulation_controllers/core/arm_controller_params.h"
#include "hy_manipulation_controllers/kinematics/kinematics_solver.h"
#include "hy_manipulation_controllers/motion_controller/motion_controller_base.h"
#include "hy_manipulation_controllers/utils/config.h"
#include <hy_hardware_interface/test_dm_4dof_state.h>
#include <hy_hardware_interface/test_dm_4dof_control.h>
namespace hy_manipulation_controllers
{

    /**
     * @brief 机械臂控制器状态枚举
     */
    enum ArmControllerState
    {
        AC_STATE_STANDBY = 0,
        AC_STATE_RUNNING,
        AC_STATE_ERROR,
        AC_STATE_DISABLE,
    };

    class ArmController
    {
    public:
        typedef std::shared_ptr<ArmController> Ptr;

    public:
        /**
         * @brief 构造函数
         *
         * @param _namespace 机械臂命名空间（话题前缀）
         * @param _param_folder 机械臂参数文件夹
         */
        ArmController(const std::string &_namespace,
                      const std::string &_param_folder);

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

        bool Initialize();

        std::string ToString(ArmControllerState state)
        {
            switch (state)
            {
            case AC_STATE_DISABLE:
                return "DISABLE";
            case AC_STATE_STANDBY:
                return "STANDBY";
            case AC_STATE_RUNNING:
                return "RUNNING";
            case AC_STATE_ERROR:
                return "ERROR";
            default:
                return "UNKNOWN";
            }
        }

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
        void
        DoJointPositionControl(const Eigen::VectorXf &_target_joint_positions,
                               const float &_vel_percentage,
                               const float &_acc_duration,
                               const float &_stiffness, const bool &_block_flag);

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
            const std::vector<Eigen::VectorXf> &_target_trajectory_positions,
            const float &_vel_percentage, const float &_acc_duration,
            const float &_stiffness, const bool &_block_flag);

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
            const hy_common::geometry::Transform3D &_target_pose,
            const float &_max_cartesian_vel, const float &_acc_duration,
            const float &_stiffness, const bool &_block_flag);

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
            const std::vector<hy_common::geometry::Transform3D> &
                _target_trajectory_poses,
            const float &_max_cartesian_vel, const float &_acc_duration,
            const float &_stiffness, const bool &_block_flag);

        /**
         * @brief 启动拖拽模式（重力补偿，用于示教）
         *
         * @param _stiffness 刚度(0.0~1.0)
         */
        void StartDragMode(const float &_stiffness);

        /**
         * @brief 停止拖拽模式
         */
        void StopDragMode();

        /**
         * @brief 启动静止模式（机械臂静止在当前位置）
         *
         * @param _stiffness 刚度(0.0~1.0)
         */
        void StartStaticMode(const float &_stiffness);

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
        void StartJoggingMode(const float &_vel_percentage, const float &_stiffness);

        /**
         * @brief 设置点动关节目标
         *
         * @param _joint_target 关节目标
         */
        void SetJoggingJointTarget(const Eigen::VectorXf &_joint_target);

        /**
         * @brief 设置点动末端位姿目标
         *
         * @param _pose_target 末端位姿目标
         */
        void SetJoggingPoseTarget(
            const hy_common::geometry::Transform3D &_pose_target);

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
        bool SolveFK(const Eigen::VectorXf &_joint_positions_in,
                     hy_common::geometry::Transform3D &_end_pose_out);

        /**
         * @brief 逆运动学求解
         *
         * @param _end_pose_in 末端位姿
         * @param _joint_positions_out 关节位置
         */
        bool SolveIK(const hy_common::geometry::Transform3D &_end_pose_in,
                     Eigen::VectorXf &_joint_positions_out);

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
            const hy_common::geometry::Point3D &_end_position_in,
            const std::vector<Eigen::VectorXf> &_joint_lower_limits,
            const std::vector<Eigen::VectorXf> &_joint_upper_limits,
            const float &_end_orientation_sample_step,
            std::vector<Eigen::VectorXf> &_solutions);

    private:
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

        /**
         * @brief 更新运动控制器状态
         *
         */
        void UpdateMotionControllerState();

        /**
         * @brief 更新机械臂控制器状态
         */
        void UpdateArmControllerState();

        /**
         * @brief 更新机械臂状态消息(接收)
         */
        void UpdateArmStateMsgs();

        /**
         * @brief 更新机械臂控制消息(发送)
         */
        void PublishArmControlMsgs();

        /**
         * @brief 更新机械臂TF
         */
        void UpdateArmTf();

        void JointStateCallback(const hy_hardware_interface::test_dm_4dof_state::ConstPtr &msg);

    private:
        std::unique_ptr<ros::NodeHandle> nh_;
        std::mutex joints_mutex_;
        std::shared_ptr<MotionControllerBase> motion_controller_current_;
        std::shared_ptr<MotionControllerBase> motion_controller_next_;

        std::shared_ptr<KinematicsSolver> kinematics_solver_;

        std::thread control_thread_;   // 控制线程
        std::thread state_pub_thread_; // 状态发布线程

        hy_common::geometry::Transform3D current_end_pose_; // 当前笛卡尔位姿

        std::string urdf_path;
        std::string cam_params_json_path;
        std::string joint_params_json_path;

        ArmControllerState control_state_;

        std::vector<JointState> joint_states_;
        std::vector<JointControlCommand> joint_control_commands_;

        ros::Subscriber joint_states_sub_;
        ros::Publisher joint_cmds_pub_;

        bool joint_state_received_ = false;
        struct JointStateHistoryEntry
        {
            ros::Time timestamp;
            std::vector<JointState> joint_states;
        };
        std::deque<JointStateHistoryEntry> joint_state_history_;

        JointTrajectory current_trajectory_;

        std::vector<Eigen::VectorXf> trajectory_joints;
    };

} // namespace hy_manipulation_controllers
