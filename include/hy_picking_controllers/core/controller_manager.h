#pragma once

#include <hy_common/logger/logger.h>
#include <ros/ros.h>

#include <memory>
#include <string>

namespace hy_picking_controllers {

class ControllerManager {
 public:
  /**
   * @brief 构造函数
   * @param _nh ROS节点句柄
   * @param _nh_private ROS私有节点句柄
   */
  ControllerManager(ros::NodeHandle& _nh, ros::NodeHandle& _nh_private);

  /**
   * @brief 析构函数
   */
  ~ControllerManager();

 private:
  ros::NodeHandle nh_;          ///< ROS节点句柄
  ros::NodeHandle nh_private_;  ///< ROS私有节点句柄
};

}  // namespace hy_picking_controllers
