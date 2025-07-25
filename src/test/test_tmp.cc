#include <hy_common/logger/logger.h>
#include <ros/ros.h>

#include "hy_common/geometry/core/transform.h"
#include "hy_manipulation_controllers/core/arm_controller.h"
using namespace hy_manipulation_controllers;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_arm_controller");
  ros::NodeHandle nh;

  float max_cartesian_vel = 0.1;
  float acc_duration = 1.0;
  float stiffness = 0.5;
  bool block_flag = true;

  std::string source_dir = std::string(DK_SOURCE_DIR);
  std::string arm_param_folder = source_dir + "/params/";
  hy_common::logger::Logger::GetInstance().Init(
      hy_common::logger::LogLevel::INFO, hy_common::logger::LogLevel::INFO,
      "controller_manager_node.log", 1 * 1024 * 1024, 10);

  ArmController::Ptr arm_controller =
      std::make_shared<ArmController>("scara_arm", arm_param_folder);

  ros::Rate loop_rate(200);
  arm_controller->Connect();

  std::vector<KinematicsSolver::JointLinkInfo> link_infos;

  if (arm_controller->GetLinkInfo(link_infos)) {
    LOG_INFO("Successfully retrieved link info.");
    LOG_INFO("link_infos.size() = {}", link_infos.size());
    for (const auto &info : link_infos) {
      LOG_INFO(
          "Joint: {}, Child Link: {}, Max Vel: {}, Lower Limit: {}, "
          "Upper Limit: {}, Link Length: {}",
          info.joint_name, info.child_link_name, info.max_vel,
          info.joint_lower_limit, info.joint_upper_limit, info.link_length);
    }
  } else {
    LOG_ERROR("Failed to retrieve link info.");
  }

  while (ros::ok()) {
    ros::spinOnce();

    // std::vector<JointState> current_joints =
    //     arm_controller->get_latest_joint_states();
    // hy_common::geometry::Transform3D end_pose =
    //     arm_controller->get_current_end_effector_pose();
    // if (!current_joints.empty()) {
    //   LOG_INFO("current_joints position: {} vel: {}",
    //            current_joints[0].position, current_joints[0].velocity);
    //   std::stringstream ss;
    //   ss << end_pose;
    //   LOG_INFO("End Pose: {}", ss.str());
    // } else {
    //   LOG_INFO("Waiting for joint states...");
    // }

    loop_rate.sleep();
  }

  arm_controller->Disconnect();
  return 0;
}
