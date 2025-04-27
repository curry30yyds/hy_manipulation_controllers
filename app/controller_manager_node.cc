#include <hy_common/logger/logger.h>
#include <ros/ros.h>

#include "hy_picking_controllers/core/controller_manager.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_manager_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  hy_common::logger::Logger::GetInstance().Init(
      hy_common::logger::LogLevel::INFO, hy_common::logger::LogLevel::INFO,
      "controller_manager_node.log", 1 * 1024 * 1024, 10);

  LOG_INFO("-------------------------------");
  LOG_INFO("Controller manager node started");

  hy_picking_controllers::ControllerManager controller_manager(nh, nh_private);

  ros::Rate loop_rate(10);  // 10Hz

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
