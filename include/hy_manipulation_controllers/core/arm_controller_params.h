#pragma once

#include <Eigen/Core>
#include <string>

namespace hy_manipulation_controllers {

class JointControlParams {
 public:
  int id;
  std::string type;  // "LINEAR" or "ROTATIONAL"
  float reduction_ratio;
  float max_vel;             // rad/s or m/s
  float max_acc;             // rad/s^2 or m/s^2
  std::string control_mode;  // "POSITION" or "VELOCITY" or "TORQUE" or "MIT"
};

class CameraExtrinsicParams {
 public:
  std::string frame_id;
  std::string child_frame_id;
  Eigen::Vector3f translation;
  Eigen::Vector3f rotation;
};

}  // namespace hy_manipulation_controllers