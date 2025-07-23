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

  // hy_common::logger::Logger::GetInstance().Init(
  //     hy_common::logger::LogLevel::INFO, hy_common::logger::LogLevel::INFO,
  //     "controller_manager_node.log", 1 * 1024 * 1024, 10);

  LOG_INFO("--------------------------------");
  LOG_INFO("Test my scara controller started");
  LOG_INFO("--------------------------------");

  ArmController::Ptr arm_controller =
      std::make_shared<ArmController>("scara_arm", arm_param_folder);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  arm_controller->Connect();

  const std::vector<std::pair<double, double>> joint_limits = {
      {0.0, 0.4},   // Joint 1: 0.0 to 0.5
      {-1.3, 1.3},  // Joint 2: -1.57 to 1.57
      {-2.0, 2.0},  // Joint 3: -2.35 to 2.35
      {-2.0, 2.0}   // Joint 4: -2.35 to 2.35
  };

  std::random_device rd;
  std::mt19937 generator(rd());

  std::vector<hy_common::geometry::Transform3D> target_trajectory_poses;

  //随机点测试
  // int numberOfPoses = 20;

  // std::cout << "Generating " << numberOfPoses
  //           << " random poses for the trajectory..." << std::endl;
  // std::cout << std::fixed << std::setprecision(2);

  // for (int i = 0; i < numberOfPoses; ++i) {
  //   Eigen::VectorXf joint_angles(4);

  //   for (size_t j = 0; j < joint_limits.size(); ++j) {
  //     std::uniform_real_distribution<double> distribution(
  //         joint_limits[j].first, joint_limits[j].second);

  //     joint_angles(j) = static_cast<float>(distribution(generator));
  //   }

  //   std::cout << "Generated Pose " << i + 1 << " Angles: [" <<
  //   joint_angles(0)
  //             << ", " << joint_angles(1) << ", " << joint_angles(2) << ", "
  //             << joint_angles(3) << "]" << std::endl;

  //   hy_common::geometry::Transform3D end_pose;
  //   if (arm_controller->SolveFK(joint_angles, end_pose)) {
  //     target_trajectory_poses.push_back(end_pose);
  //   } else {
  //     LOG_WARN(
  //         "Forward Kinematics failed for a randomly generated
  //         pose.Skipping.");
  //   }
  // }

  //定点测试
  std::vector<Eigen::VectorXf> fixed_joint_angles = {
      (Eigen::Vector4f() << 0.30f, -1.0f, 1.1f, -1.8f).finished(),
      (Eigen::Vector4f() << 0.36f, -0.01f, -1.68f, 0.24f).finished(),
      (Eigen::Vector4f() << 0.15f, -1.08f, -1.80f, -1.07f).finished(),
      (Eigen::Vector4f() << 0.35f, -0.60f, 1.95f, -1.55f).finished(),
      (Eigen::Vector4f() << 0.04f, 0.32f, 1.94f, 0.10f).finished()};

  for (size_t i = 0; i < fixed_joint_angles.size(); ++i) {
    const auto& joint_angles = fixed_joint_angles[i];

    std::cout << "Pose " << i + 1 << " Angles: [" << joint_angles(0) << ", "
              << joint_angles(1) << ", " << joint_angles(2) << ", "
              << joint_angles(3) << "]" << std::endl;

    hy_common::geometry::Transform3D end_pose;
    if (arm_controller->SolveFK(joint_angles, end_pose)) {
      target_trajectory_poses.push_back(end_pose);
    } else {
      LOG_WARN("Forward Kinematics failed for the specified pose. Skipping.");
    }
  }

  std::cout << ">>> Press Enter to start Cartesian trajectory control"
            << std::endl;
  std::string dummy_input;
  std::getline(std::cin, dummy_input);

  LOG_INFO("Sending Cartesian trajectory command...");
  ros::Duration(0.5).sleep();

  arm_controller->DoCartesianTrajectoryControl(target_trajectory_poses, 0.8f,
                                               0.5f, 100.0f, false);

  ros::waitForShutdown();

  arm_controller->Disconnect();
  return 0;
}
