#include <ros/ros.h>
#include "hy_manipulation_controllers/kinematics/kinematics_solver.h"
#include <ros/package.h>
#include <kdl/frames_io.hpp>

using namespace hy_manipulation_controllers;

void printJntArray(const KDL::JntArray &joints, const std::string &title)
{
    std::stringstream ss;
    ss << title << ": [";
    for (unsigned int i = 0; i < joints.rows(); ++i)
    {
        ss << joints(i);
        if (i != joints.rows() - 1)
            ss << ", ";
    }
    ss << "]";
    LOG_INFO("{}", ss.str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_kinematics_solver");
    ros::NodeHandle nh("~");
    // 参数获取
    std::string package_path = ros::package::getPath("hy_manipulation_controllers");
    if (package_path.empty())
    {
        LOG_ERROR("Failed to find package path for my_robot_description");
        return -1;
    }

    std::string urdf_path = package_path + "/urdf/panda_arm.urdf";
    LOG_INFO("URDF path: [{}] ", urdf_path);

    // 求解器构造
    std::shared_ptr<hy_manipulation_controllers::KinematicsSolver> solver;
    try
    {
        solver = std::make_shared<hy_manipulation_controllers::KinematicsSolver>(urdf_path);
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("Failed to create KinematicsSolver: {}", e.what());
        return -1;
    }

    unsigned int num_joints = solver->GetChain().getNrOfJoints();

    // FK
    LOG_INFO("\n\n>>>>>>>>>> TESTING FORWARD KINEMATICS (FK) >>>>>>>>>>");

    Eigen::VectorXf joint_angles(num_joints);
    for (size_t i = 0; i < num_joints; ++i)
    {
        joint_angles(i) = 0.1f * (i + 1); // 示例数据
    }

    // 求解FK
    Eigen::Matrix4f end_pose;
    if (solver->SolveFK(joint_angles, end_pose))
    {
        std::cout << "\n[FK] Input Joint Angles:\n"
                  << joint_angles.transpose() << std::endl;
        std::cout << "\n[FK] Resulting End Effector Pose:\n"
                  << end_pose << std::endl;
    }
    else
    {
        LOG_ERROR("SolveFK failed!");
    }

    // IK
    LOG_INFO("\n\n>>>>>>>>>> TESTING INVERSE KINEMATICS (IK) >>>>>>>>>>");
    Eigen::VectorXf ik_solution_joints(num_joints);

    Eigen::VectorXf joint_positions_;
    joint_positions_ = Eigen::VectorXf::Zero(num_joints);

    if (solver->SolveIK(end_pose, joint_positions_, ik_solution_joints))
    {
        std::cout << "IK succeeded! Joint values:\n"
                  << ik_solution_joints.transpose() << std::endl;
    }
    else
    {
        LOG_ERROR("IK Test Failed!");
    }

    // 轨迹插值

    LOG_INFO("\n\n>>>>>>>>>> TESTING TRAJECTORY INTERPOLATION >>>>>>>>>>");
    int num_steps = 5;
    double duration = 5;
    std::vector<KDL::JntArray> trajectory;

    if (solver->InterpolateTrajectory(joint_positions_, ik_solution_joints, trajectory, duration, num_steps))
    {
        LOG_INFO("--- Resulting Trajectory ({} steps) ---", trajectory.size());
        for (size_t i = 0; i < trajectory.size(); ++i)
        {
            printJntArray(trajectory[i], "Step " + std::to_string(i));
        }
    }
    else
    {
        LOG_ERROR("Trajectory Interpolation Test Failed!");
    }

    ros::spinOnce();

    return 0;
}
