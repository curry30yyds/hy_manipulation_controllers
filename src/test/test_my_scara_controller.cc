#include <hy_common/logger/logger.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <iomanip>
#include <iostream>
#include <random>

#include "hy_manipulation_controllers/core/arm_controller.h"

using namespace hy_manipulation_controllers;

int main(int argc, char** argv) {
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

  // 构造目标末端轨迹
  // std::vector<hy_common::geometry::Transform3D> target_trajectory_poses;
  // for (int i = 1; i <= 4; ++i) {
  //   Eigen::VectorXf joint_angles(4);
  //   for (size_t j = 0; j < 4; ++j) {
  //     joint_angles(j) = 0.1f * (j + 1.0 * i);
  //   }
  //   //实际的角度
  //   // 0：[0.15,0.30,0.45,0.60]
  //   // 1：[0.25,0.40,0.55,0.70]
  //   // 2：[0.35,0.50,0.65,0.80]
  //   // 3：[0.45,0.60,0.75,0.90]
  //   hy_common::geometry::Transform3D end_pose;
  //   if (arm_controller->SolveFK(joint_angles, end_pose)) {
  //     target_trajectory_poses.push_back(end_pose);
  //   }
  // }

  const std::vector<std::pair<double, double>> joint_limits = {
      {0.0, 0.4},   // Joint 1: 0.0 to 0.5
      {-1.3, 1.3},  // Joint 2: -1.57 to 1.57
      {-2.0, 2.0},  // Joint 3: -2.35 to 2.35
      {-2.0, 2.0}   // Joint 4: -2.35 to 2.35
  };

  std::random_device rd;
  std::mt19937 generator(rd());

  std::vector<hy_common::geometry::Transform3D> target_trajectory_poses;
  int numberOfPoses = 20;

  std::cout << "Generating " << numberOfPoses
            << " random poses for the trajectory..." << std::endl;
  std::cout << std::fixed << std::setprecision(2);

  for (int i = 0; i < numberOfPoses; ++i) {
    Eigen::VectorXf joint_angles(4);

    for (size_t j = 0; j < joint_limits.size(); ++j) {
      std::uniform_real_distribution<double> distribution(
          joint_limits[j].first, joint_limits[j].second);

      joint_angles(j) = static_cast<float>(distribution(generator));
    }

    std::cout << "Generated Pose " << i + 1 << " Angles: [" << joint_angles(0)
              << ", " << joint_angles(1) << ", " << joint_angles(2) << ", "
              << joint_angles(3) << "]" << std::endl;

    hy_common::geometry::Transform3D end_pose;
    if (arm_controller->SolveFK(joint_angles, end_pose)) {
      target_trajectory_poses.push_back(end_pose);
    } else {
      LOG_WARN(
          "Forward Kinematics failed for a randomly generated pose.Skipping.");
    }
  }

  std::cout << ">>> Press Enter to start Cartesian trajectory control"
            << std::endl;
  std::string dummy_input;
  std::getline(std::cin, dummy_input);

  LOG_INFO("Sending Cartesian trajectory command...");
  ros::Duration(0.5).sleep();

  arm_controller->DoCartesianTrajectoryControl(target_trajectory_poses, 2.0f,
                                               0.5f, 100.0f, false);

  ros::waitForShutdown();

  arm_controller->Disconnect();
  return 0;
}
