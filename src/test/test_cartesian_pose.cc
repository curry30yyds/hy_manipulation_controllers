#include <hy_common/logger/logger.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <iomanip>
#include <iostream>
#include <random>

#include "hy_manipulation_controllers/core/arm_controller.h"

using namespace hy_manipulation_controllers;

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_cartesian_pose");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string source_dir = std::string(DK_SOURCE_DIR);
  std::string arm_param_folder = source_dir + "/params/";

  //   hy_common::logger::Logger::GetInstance().Init(
  //       hy_common::logger::LogLevel::INFO, hy_common::logger::LogLevel::INFO,
  //       "controller_manager_node.log", 1 * 1024 * 1024, 10);

  LOG_INFO("--------------------------------");
  LOG_INFO("Test my scara controller started");
  LOG_INFO("--------------------------------");

  ArmController::Ptr arm_controller =
      std::make_shared<ArmController>("scara_arm", arm_param_folder);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  arm_controller->Connect();

  // 构造目标末端轨迹
  Eigen::VectorXf joint_angles(4);
  joint_angles << 0.30f, -1.0f, 1.1f, -1.8f;

  hy_common::geometry::Transform3D end_pose;
  if (!arm_controller->SolveFK(joint_angles, end_pose)) {
    LOG_ERROR("Failed to solve FK for the specified joint angles.");
    return -1;
  }

  std::cout << ">>> Press Enter to start Cartesian trajectory control"
            << std::endl;
  std::string dummy_input;
  std::getline(std::cin, dummy_input);

  LOG_INFO("Sending Cartesian trajectory command...");
  ros::Duration(0.5).sleep();

  arm_controller->DoCartesianPoseControl(end_pose, 1.0f, 0.5f, 100.0f, false);

  ros::waitForShutdown();

  arm_controller->Disconnect();
  return 0;
}
