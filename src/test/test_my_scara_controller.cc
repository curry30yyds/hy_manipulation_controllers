#include <hy_common/logger/logger.h>
#include <ros/ros.h>
#include "hy_manipulation_controllers/core/arm_controller.h"

using namespace hy_manipulation_controllers;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_my_scara_controller");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::string source_dir = std::string(DK_SOURCE_DIR);
    std::string arm_param_folder = source_dir + "/params/";

    hy_common::logger::Logger::GetInstance().Init(
        hy_common::logger::LogLevel::INFO, hy_common::logger::LogLevel::INFO,
        "controller_manager_node.log", 1 * 1024 * 1024, 10);

    LOG_INFO("--------------------------------");
    LOG_INFO("Test standard scara controller started");
    LOG_INFO("--------------------------------");

    ArmController::Ptr arm_controller =
        std::make_shared<ArmController>("scara_arm", arm_param_folder);

    arm_controller->Connect();

    ros::Duration(0.5).sleep();

    std::vector<hy_common::geometry::Transform3D> target_trajectory_poses;

    for (int i = 1; i <= 4; ++i)
    {
        Eigen::VectorXf joint_angles(4);
        for (size_t j = 0; j < 4; ++j)
        {
            joint_angles(j) = 0.1f * (j + i);
        }

        hy_common::geometry::Transform3D end_pose;
        if (arm_controller->SolveFK(joint_angles, end_pose))
        {
            target_trajectory_poses.push_back(end_pose);
        }
    }

    LOG_INFO("Sending Cartesian trajectory command...");

    arm_controller->DoCartesianTrajectoryControl(target_trajectory_poses, 0.5f, 0.2f, 100.0f, false);

    ros::Rate loop_rate(10); // 10Hz
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    arm_controller->Disconnect();
    return 0;
}
