#include <hy_common/logger/logger.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include "hy_manipulation_controllers/core/arm_controller.h"

using namespace hy_manipulation_controllers;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_drag_controller");

  ros::NodeHandle nh;
  std::string source_dir = std::string(DK_SOURCE_DIR);
  std::string arm_param_folder = source_dir + "/params/";

  hy_common::logger::Logger::GetInstance().Init(
      hy_common::logger::LogLevel::INFO, hy_common::logger::LogLevel::INFO,
      "drag_mode_test.log", 1 * 1024 * 1024, 10);

  LOG_INFO("--------------------------------");
  LOG_INFO("Drag Mode Test Program Started");
  LOG_INFO("--------------------------------");

  ArmController::Ptr arm_controller =
      std::make_shared<ArmController>("scara_arm", arm_param_folder);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  arm_controller->Connect();

  ros::Duration(1.0).sleep();
  std::cout << ">>> Press Enter to start to move " << std::endl;
  std::string dummy_input;
  std::getline(std::cin, dummy_input);

  arm_controller->StartDragMode(0.0f);

  //   ros::Duration(20.0).sleep();

  // LOG_INFO("STOPPING DRAG MODE. Arm will now hold its final position.");
  // 停止拖拽 就进入静止模式

  //   arm_controller->StopDragMode();

  ros::waitForShutdown();
  //   arm_controller->Disconnect();
  return 0;
}
