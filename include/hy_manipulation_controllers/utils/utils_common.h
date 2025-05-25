#pragma once

#include <string>
#include <vector>

namespace hy_manipulation_controllers {

/**
 * @brief 关节轨迹点
 */
typedef struct {
  int id;
  float position;
  float velocity;
  float acceleration;
  float timestamp;
} JointTrajectoryPoint;

/**
 * @brief 关节轨迹
 */
typedef std::vector<JointTrajectoryPoint> JointTrajectory;

/**
 * @brief 关节状态
 */
typedef struct {
  int id;
  float position;
  float velocity;
  float torque;
  float temperature;
  float current;
  float voltage;
  int running_state;
  int error_type;
} JointState;

/**
 * @brief 关节控制命令
 */
typedef struct {
  int id;
  std::string control_mode;  // "POSITION" or "VELOCITY" or "TORQUE" or "MIT"

  float target_position;
  float target_velocity;
  float target_torque;

  float mit_kp;
  float mit_kd;
  float mit_t_ff;
} JointControlCommand;

}  // namespace hy_manipulation_controllers
