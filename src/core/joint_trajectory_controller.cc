#include "hy_manipulation_controllers/motion_controller/joint_trajectory_controller.h"
#include <algorithm>
#include <chrono>
#include <thread>

namespace hy_manipulation_controllers
{

    JointTrajectoryController::JointTrajectoryController(const std::vector<JointControlParams> &_joint_control_params)
        : MotionControllerBase(_joint_control_params)
    {
    }

    JointTrajectoryController::~JointTrajectoryController()
    {
    }

    void JointTrajectoryController::SetTrajectory(const JointTrajectory &_joint_trajectory)
    {
        // 保存关节轨迹点序列副本
        joint_trajectory_ = _joint_trajectory;

        // 按照时间戳对轨迹点排序，确保时间顺序
        std::sort(joint_trajectory_.begin(), joint_trajectory_.end(),
                  [](const JointTrajectoryPoint &a, const JointTrajectoryPoint &b)
                  {
                      return a.timestamp < b.timestamp;
                  });

        // 重置控制器状态为已停止，准备新的轨迹
        motion_controller_state_ = MCS_STOPPED;
    }

    void JointTrajectoryController::Start(bool _block_flag)
    {
        if (joint_trajectory_.empty())
        {
            // 如果没有设置轨迹就调用Start，标记错误状态
            motion_controller_state_ = MCS_ERROR;
            return;
        }

        // 将控制器状态设置为运行中
        motion_controller_state_ = MCS_RUNNING;
        // 记录轨迹启动时刻
        auto start_time = std::chrono::steady_clock::now();

        if (_block_flag)
        {
            // 若需要阻塞等待，则循环等待运动完成
            while (motion_controller_state_ == MCS_RUNNING || motion_controller_state_ == MCS_STOPPING)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    void JointTrajectoryController::Update(const std::vector<JointState> &_joint_states,
                                           std::vector<JointControlCommand> &_joint_control_commands)
    {
        if (motion_controller_state_ == MCS_STOPPED || motion_controller_state_ == MCS_ERROR)
        {
            // 若控制器已停止或错误，则不输出控制命令
            _joint_control_commands.clear();
            return;
        }

        // 获取当前时间距离轨迹开始的运行时间
        static auto traj_start_time = std::chrono::steady_clock::now();
        static bool first_call = true;
        if (first_call)
        {
            // 记录第一次进入Update的时间作为轨迹起点时间
            traj_start_time = std::chrono::steady_clock::now();
            first_call = false;
        }
        std::chrono::duration<float> elapsed = std::chrono::steady_clock::now() - traj_start_time;
        float current_time = elapsed.count(); // 当前已运行时间（秒）

        _joint_control_commands.clear();
        _joint_control_commands.reserve(_joint_states.size());

        // 遍历每个关节的当前状态，计算对应的控制命令
        for (const JointState &state : _joint_states)
        {
            int joint_id = state.id;
            float actual_pos = state.position;
            float actual_vel = state.velocity;

            // 在轨迹中查找该关节当前时间对应的目标点（插值）
            // 找到当前关节在 joint_trajectory_ 中相邻的前后两个轨迹点
            JointTrajectoryPoint prev_pt = {joint_id, actual_pos, 0.0f, 0.0f, 0.0f};
            JointTrajectoryPoint next_pt = {joint_id, actual_pos, 0.0f, 0.0f, 0.0f};
            bool found_prev = false, found_next = false;
            for (const JointTrajectoryPoint &pt : joint_trajectory_)
            {
                if (pt.id != joint_id)
                    continue;
                if (pt.timestamp <= current_time)
                {
                    // 找到不超过当前时间的最新轨迹点
                    if (!found_prev || pt.timestamp > prev_pt.timestamp)
                    {
                        prev_pt = pt;
                        found_prev = true;
                    }
                }
                if (pt.timestamp >= current_time)
                {
                    // 找到第一个晚于当前时间的轨迹点
                    if (!found_next || pt.timestamp < next_pt.timestamp)
                    {
                        next_pt = pt;
                        found_next = true;
                    }
                }
            }
            if (!found_prev)
            {
                // 当前时间早于该关节轨迹起点，则使用轨迹起始点作为前点
                for (const JointTrajectoryPoint &pt : joint_trajectory_)
                {
                    if (pt.id == joint_id)
                    {
                        prev_pt = pt;
                        found_prev = true;
                        break;
                    }
                }
            }
            if (!found_next)
            {
                // 当前时间超出该关节轨迹末尾，则使用最后一个点作为后点
                for (auto it = joint_trajectory_.rbegin(); it != joint_trajectory_.rend(); ++it)
                {
                    if (it->id == joint_id)
                    {
                        next_pt = *it;
                        found_next = true;
                        break;
                    }
                }
            }

            // 确定用于插值的前后轨迹点时间
            float t0 = prev_pt.timestamp;
            float t1 = next_pt.timestamp;
            float desired_pos = prev_pt.position;
            float desired_vel = prev_pt.velocity;
            if (fabs(t1 - t0) > 1e-6f && current_time >= t0 && current_time <= t1)
            {
                // 进行插值计算
                float ratio = (current_time - t0) / (t1 - t0);
                // 使用线性插值计算期望位置和速度   后面再加
                desired_pos = prev_pt.position + ratio * (next_pt.position - prev_pt.position);
                desired_vel = prev_pt.velocity + ratio * (next_pt.velocity - prev_pt.velocity);
            }
            else if (current_time > t1)
            {
                // 当前时间超过最后一个轨迹点时间，使用最后点
                desired_pos = next_pt.position;
                desired_vel = 0.0f; // 轨迹结束后速度为0
            }
            else if (current_time < t0)
            {
                // 当前时间在第一点之前，使用第一点
                desired_pos = prev_pt.position;
                desired_vel = prev_pt.velocity;
            }

            // 计算当前位置误差
            float pos_error = desired_pos - actual_pos;
            float vel_error = desired_vel - actual_vel;

            // 根据关节的控制模式生成控制命令
            JointControlCommand cmd;
            cmd.id = joint_id;
            // 查找该关节的控制参数以获取控制模式和限制

            std::string mode = "POSITION";
            float max_vel;
            float kp = 0.0f;
            float kd = 0.0f;
            for (const JointControlParams &param : joint_control_params_)
            {
                if (param.id == joint_id)
                {
                    mode = param.control_mode;
                    max_vel = param.max_vel;
                    break;
                }
            }

            if (fabs(desired_vel) > max_vel)
            {
                // 限幅速度
                desired_vel = (desired_vel > 0 ? 1 : -1) * max_vel;
            }

            if (mode == "POSITION")
            {
                // 位置模式
                cmd.control_mode = "POSITION";
                cmd.target_position = desired_pos;
                cmd.target_velocity = 0.0f;
                cmd.target_torque = 0.0f;
                cmd.mit_kp = 0.0f;
                cmd.mit_kd = 0.0f;
                cmd.mit_t_ff = 0.0f;
            }
            else if (mode == "VELOCITY")
            {

                cmd.control_mode = "VELOCITY";

                float vel_command = desired_vel + 0.0f * pos_error;
                // 限制速度命令不超过最大速度
                if (vel_command > max_vel)
                    vel_command = max_vel;
                if (vel_command < -max_vel)
                    vel_command = -max_vel;
                cmd.target_velocity = vel_command;
                cmd.target_position = 0.0f;
                cmd.target_torque = 0.0f;
                cmd.mit_kp = 0.0f;
                cmd.mit_kd = 0.0f;
                cmd.mit_t_ff = 0.0f;
            }
            else if (mode == "TORQUE")
            {

                cmd.control_mode = "TORQUE";

                float torque_command = kp * pos_error + kd * vel_error;

                cmd.target_torque = torque_command;
                cmd.target_position = 0.0f;
                cmd.target_velocity = 0.0f;
                cmd.mit_kp = 0.0f;
                cmd.mit_kd = 0.0f;
                cmd.mit_t_ff = 0.0f;
            }
            else if (mode == "MIT")
            {
                cmd.control_mode = "MIT";
                cmd.target_position = desired_pos;
                cmd.target_velocity = desired_vel;

                cmd.mit_t_ff = 0.0f;

                cmd.mit_kp = kp;
                cmd.mit_kd = kd;

                cmd.target_torque = 0.0f;
            }
            else
            {
                cmd.control_mode = "POSITION";
                cmd.target_position = desired_pos;
                cmd.target_velocity = 0.0f;
                cmd.target_torque = 0.0f;
                cmd.mit_kp = 0.0f;
                cmd.mit_kd = 0.0f;
                cmd.mit_t_ff = 0.0f;
            }

            _joint_control_commands.push_back(cmd);
        }

        // 检查是否到达轨迹终点来更新状态
        float max_time = 0.0f;
        for (const JointTrajectoryPoint &pt : joint_trajectory_)
        {
            if (pt.timestamp > max_time)
            {
                max_time = pt.timestamp;
            }
        }
        if (current_time >= max_time)
        {
            // 如果当前时间已经超过轨迹时间，则认为轨迹完成，进入停止状态
            motion_controller_state_ = MCS_STOPPED;
        }
    }
    void JointTrajectoryController::Stop(float _duration)
    {
        if (motion_controller_state_ != MCS_RUNNING)
        {
            // 非运行状态下调用Stop，无需处理
            return;
        }
        // 将控制器状态设置为减速中
        motion_controller_state_ = MCS_STOPPING;

        // 计算各关节需要的减速时间
        float required_time = 0.0f;
        // 准备新的减速轨迹序列
        JointTrajectory decel_traj;

        std::vector<JointState> current_states;

        // 计算实际需要的减速时间
        for (const JointState &state : current_states)
        {
            // 查找对应关节参数的最大加速度限制
            float max_acc = 0.0f;
            for (const JointControlParams &param : joint_control_params_)
            {
                if (param.id == state.id)
                {
                    max_acc = param.max_acc;
                    break;
                }
            }
            if (max_acc <= 0)
            {
                continue;
            }
            float time_to_stop = fabs(state.velocity) / max_acc;
            if (time_to_stop > required_time)
            {
                required_time = time_to_stop;
            }
        }

        if (required_time < _duration)
        {
            required_time = _duration;
        }

        // 生成减速轨迹点：当前时刻和 required_time 时刻的状态
        for (const JointState &state : current_states)
        {
            JointTrajectoryPoint start_pt;
            start_pt.id = state.id;
            start_pt.position = state.position;
            start_pt.velocity = state.velocity;
            start_pt.acceleration = 0.0f;
            start_pt.timestamp = 0.0f; // 当前时刻作为0

            JointTrajectoryPoint end_pt;
            end_pt.id = state.id;
            end_pt.position = state.position + state.velocity * required_time * 0.5f; // 线性减速假设下移动的距离
            end_pt.velocity = 0.0f;
            end_pt.acceleration = 0.0f;
            end_pt.timestamp = required_time; // 在required_time时速度降为0

            decel_traj.push_back(start_pt);
            decel_traj.push_back(end_pt);
        }

        // 将当前轨迹替换为减速轨迹，并按时间排序
        joint_trajectory_ = decel_traj;
        std::sort(joint_trajectory_.begin(), joint_trajectory_.end(),
                  [](const JointTrajectoryPoint &a, const JointTrajectoryPoint &b)
                  {
                      return a.timestamp < b.timestamp;
                  });
    }

} // namespace hy_manipulation_controllers
