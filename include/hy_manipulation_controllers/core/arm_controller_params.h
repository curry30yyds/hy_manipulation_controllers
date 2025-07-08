#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>
#include <hy_common/logger/logger.h>

namespace hy_manipulation_controllers
{

  class JointControlParams
  {
  public:
    JointControlParams()
    {
      id = 0;
      type = "ROTATIONAL";       // 默认类型为旋转关节
      reduction_ratio = 1.0f;    // 减速比默认1.0
      max_vel = 1.0f;            // 默认最大速度 1.0 rad/s 或 m/s
      max_acc = 2.0f;            // 默认最大加速度 2.0 rad/s² 或 m/s²
      control_mode = "POSITION"; // 默认控制模式为位置控制
    }

    JointControlParams(int _id,
                       const std::string &_type,
                       float _reduction_ratio,
                       float _max_vel,
                       float _max_acc,
                       const std::string &_control_mode)
    {
      id = _id;
      type = (_type == "LINEAR") ? "LINEAR" : "ROTATIONAL";
      reduction_ratio = _reduction_ratio;
      max_vel = _max_vel;
      max_acc = _max_acc;
      control_mode = _control_mode;
    }

  public:
    int id;
    std::string type; // "LINEAR" or "ROTATIONAL"
    float reduction_ratio;
    float max_vel;            // rad/s or m/s
    float max_acc;            // rad/s^2 or m/s^2
    std::string control_mode; // "POSITION" or "VELOCITY" or "TORQUE" or "MIT"
  };

  class CameraExtrinsicParams
  {
  public:
    CameraExtrinsicParams()
    {
      frame_id = "world";
      child_frame_id = "camera";
      translation = Eigen::Vector3f::Zero(); // 默认无平移
      rotation = Eigen::Vector3f::Zero();    // 默认无旋转
    }

    CameraExtrinsicParams(const std::string &_frame_id,
                          const std::string &_child_frame_id,
                          const Eigen::Vector3f &_translation,
                          const Eigen::Vector3f &_rotation)
    {
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
    bool loadFromJson(const std::string &filepath)
    {
      try
      {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
          LOG_ERROR("Failed to open camera extrinsic file: {}", filepath);
          return false;
        }
        nlohmann::json j;
        file >> j;

        frame_id = j.at("frame_id").get<std::string>();
        child_frame_id = j.at("child_frame_id").get<std::string>();

        std::vector<float> t = j.at("translation").get<std::vector<float>>();
        std::vector<float> r = j.at("rotation").get<std::vector<float>>();

        if (t.size() != 3 || r.size() != 3)
        {
          LOG_ERROR("Camera extrinsic file format error: translation/rotation must have 3 elements");
          return false;
        }

        translation = Eigen::Vector3f(t[0], t[1], t[2]);
        rotation = Eigen::Vector3f(r[0], r[1], r[2]);

        return true;
      }
      catch (const std::exception &e)
      {
        LOG_ERROR("Failed to load camera extrinsic param! Exception: {}", e.what());
        return false;
      }
    }
  };

} // namespace hy_manipulation_controllers
