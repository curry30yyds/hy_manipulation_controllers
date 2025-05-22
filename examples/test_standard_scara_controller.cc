#include <hy_common/logger/logger.h>
#include <ros/ros.h>

#include "hy_manipulation_controllers/core/arm_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_standard_scara_controller");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  hy_common::logger::Logger::GetInstance().Init(
      hy_common::logger::LogLevel::INFO, hy_common::logger::LogLevel::INFO,
      "controller_manager_node.log", 1 * 1024 * 1024, 10);

  LOG_INFO("--------------------------------");
  LOG_INFO("Test standard scara controller started");
  LOG_INFO("--------------------------------");

  ros::Rate loop_rate(10);  // 10Hz

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
