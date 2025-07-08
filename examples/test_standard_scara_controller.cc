#include <hy_common/logger/logger.h>
#include <ros/ros.h>

#include "hy_manipulation_controllers/core/arm_controller.h"

using namespace hy_manipulation_controllers;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_standard_scara_controller");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string source_dir = std::string(DK_SOURCE_DIR);
  std::string arm_param_folder = source_dir + "/params/"; // 加载参数，暂存
  hy_common::logger::Logger::GetInstance().Init(
      hy_common::logger::LogLevel::INFO, hy_common::logger::LogLevel::INFO,
      "controller_manager_node.log", 1 * 1024 * 1024, 10);

  LOG_INFO("--------------------------------");
  LOG_INFO("Test standard scara controller started");
  LOG_INFO("--------------------------------");
  ArmController::Ptr arm_controller =
      std::make_shared<ArmController>("scara_arm", arm_param_folder);
  ros::Rate loop_rate(10); // 10Hz
  arm_controller->Connect();
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  arm_controller->Disconnect();
  return 0;
}
