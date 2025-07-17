#pragma once

#include <hy_common/logger/logger.h>

#include <Eigen/Core>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace hy_manipulation_controllers {

class JointControlParams {
 public:
  // 默认构造函数，为增益提供了初始值
  JointControlParams() {
    id = 0;
    type = "ROTATIONAL";
    reduction_ratio = 1.0f;
    max_vel = 1.0f;
    max_acc = 2.0f;
    control_mode = "POSITION";
    mit_kp = 10.0;
    mit_kd = 1.0;
    mit_t_ff = 0.0;
    Kp = 10.0;
    Kd = 0.1;
  }
  // 暂时还没用
  JointControlParams(int _id, const std::string &_type, float _reduction_ratio,
                     float _max_vel, float _max_acc,
                     const std::string &_control_mode, float _mit_kp,
                     float _mit_kd, float _mit_t_ff) {
    id = _id;
    type = (_type == "LINEAR") ? "LINEAR" : "ROTATIONAL";
    reduction_ratio = _reduction_ratio;
    max_vel = _max_vel;
    max_acc = _max_acc;
    control_mode = _control_mode;
    mit_kp = _mit_kp;
    mit_kd = _mit_kd;
    mit_t_ff = _mit_t_ff;
  }

  static bool loadFromJson(const std::string &filepath,
                           std::vector<JointControlParams> &params_out) {
    try {
      std::ifstream file(filepath);
      if (!file.is_open()) {
        LOG_ERROR("Failed to open joint control parameters file: {}", filepath);
        return false;
      }

      nlohmann::json j;
      file >> j;

      for (const auto &joint : j.at("joints")) {
        JointControlParams p;

        p.id = joint.at("id").get<int>();
        p.type = joint.at("type").get<std::string>();
        p.reduction_ratio = joint.at("reduction_ratio").get<float>();
        p.max_vel = joint.at("max_vel").get<float>();
        p.max_acc = joint.at("max_acc").get<float>();
        p.control_mode = joint.at("control_mode").get<std::string>();
        p.mit_kp = joint.at("mit_kp").get<float>();
        p.mit_kd = joint.at("mit_kd").get<float>();
        p.mit_t_ff = joint.at("mit_t_ff").get<float>();
        p.Kp = joint.at("Kp").get<float>();
        p.Kd = joint.at("Kd").get<float>();
        params_out.push_back(p);
      }

      return true;
    } catch (const std::exception &e) {
      LOG_ERROR("Failed to load joint control parameters: {}", e.what());
      return false;
    }
  }

 public:
  int id;
  std::string type;
  float reduction_ratio;
  float max_vel;
  float max_acc;
  std::string control_mode;
  // 增益参数
  double mit_kp;
  double mit_kd;
  double mit_t_ff;
  double Kp;
  double Kd;
};

class CameraExtrinsicParams {
 public:
  CameraExtrinsicParams() {
    frame_id = "world";
    child_frame_id = "camera";
    translation = Eigen::Vector3f::Zero();  // 默认无平移
    rotation = Eigen::Vector3f::Zero();     // 默认无旋转
  }

  CameraExtrinsicParams(const std::string &_frame_id,
                        const std::string &_child_frame_id,
                        const Eigen::Vector3f &_translation,
                        const Eigen::Vector3f &_rotation) {
    frame_id = _frame_id;
    child_frame_id = _child_frame_id;
    translation = _translation;
    rotation = _rotation;
  }

 public:
  std::string frame_id;
  std::string child_frame_id;
  Eigen::Vector3f translation;
  Eigen::Vector3f rotation;

  // 从JSON文件加载相机外参
  bool loadFromJson(const std::string &filepath) {
    try {
      std::ifstream file(filepath);
      if (!file.is_open()) {
        LOG_ERROR("Failed to open camera extrinsic file: {}", filepath);
        return false;
      }
      nlohmann::json j;
      file >> j;

      frame_id = j.at("frame_id").get<std::string>();
      child_frame_id = j.at("child_frame_id").get<std::string>();

      std::vector<float> t = j.at("translation").get<std::vector<float>>();
      std::vector<float> r = j.at("rotation").get<std::vector<float>>();

      if (t.size() != 3 || r.size() != 3) {
        LOG_ERROR(
            "Camera extrinsic file format error: translation/rotation must "
            "have 3 elements");
        return false;
      }

      translation = Eigen::Vector3f(t[0], t[1], t[2]);
      rotation = Eigen::Vector3f(r[0], r[1], r[2]);

      return true;
    } catch (const std::exception &e) {
      LOG_ERROR("Failed to load camera extrinsic param! Exception: {}",
                e.what());
      return false;
    }
  }
};

}  // namespace hy_manipulation_controllers
