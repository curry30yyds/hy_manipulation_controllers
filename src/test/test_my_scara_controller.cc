#include <hy_common/logger/logger.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <iostream>

#include "hy_manipulation_controllers/core/arm_controller.h"

using namespace hy_manipulation_controllers;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_my_scara_controller");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string source_dir = std::string(DK_SOURCE_DIR);
  std::string arm_param_folder = source_dir + "/params/";

  hy_common::logger::Logger::GetInstance().Init(
      hy_common::logger::LogLevel::INFO, hy_common::logger::LogLevel::INFO,
      "controller_manager_node.log", 1 * 1024 * 1024, 10);

  LOG_INFO("--------------------------------");
  LOG_INFO("Test my scara controller started");
  LOG_INFO("--------------------------------");

  ArmController::Ptr arm_controller =
      std::make_shared<ArmController>("scara_arm", arm_param_folder);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  arm_controller->Connect();

  // 等待第一次接收到关节状态
  ros::Time start_time = ros::Time::now();
  while (!arm_controller->HasJointStateReceived()) {
    if ((ros::Time::now() - start_time).toSec() > 2.0) {
      LOG_ERROR("Timeout: No joint state received within 2 seconds.");
      return 1;
    }
    ros::Rate(100).sleep();
  }
  LOG_INFO("Joint state received, ready to proceed.");

  // 构造目标末端轨迹
  std::vector<hy_common::geometry::Transform3D> target_trajectory_poses;
  for (int i = 1; i <= 4; ++i) {
    Eigen::VectorXf joint_angles(4);
    for (size_t j = 0; j < 4; ++j) {
      joint_angles(j) = 0.1f * (j + 1.5 * i);
    }

    hy_common::geometry::Transform3D end_pose;
    if (arm_controller->SolveFK(joint_angles, end_pose)) {
      target_trajectory_poses.push_back(end_pose);
    }
  }

  std::cout << ">>> Press Enter to start Cartesian trajectory control"
            << std::endl;
  std::string dummy_input;
  std::getline(std::cin, dummy_input);

  LOG_INFO("Sending Cartesian trajectory command...");
  ros::Duration(0.5).sleep();

  arm_controller->DoCartesianTrajectoryControl(target_trajectory_poses, 0.5f,
                                               0.2f, 100.0f, false);

  ros::waitForShutdown();

  arm_controller->Disconnect();
  return 0;
}
