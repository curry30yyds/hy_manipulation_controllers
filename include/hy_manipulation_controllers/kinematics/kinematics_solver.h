#pragma once

#include <hy_common/logger/logger.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <memory>
#include <queue>
#include <random>
#include <trac_ik/trac_ik.hpp>

#include "hy_common/geometry/core/transform.h"
#include "hy_manipulation_controllers/core/arm_controller_params.h"
#include "hy_manipulation_controllers/motion_controller/joint_trajectory_controller.h"
#include "hy_manipulation_controllers/utils/config.h"
// #include "hy_manipulation_controllers/core/arm_controller.h"

namespace hy_manipulation_controllers {

class KinematicsSolver {
 public:
  typedef std::shared_ptr<KinematicsSolver> Ptr;

 public:
  /*
 四个构建函数介绍
  */
  KinematicsSolver();                          //内部构建chain
  KinematicsSolver(const KDL::Chain &_chain);  //已有chain进行初始化
  KinematicsSolver(
      const std::string &_urdf_path);  //从urdf构建 并从urdf里面寻找
  KinematicsSolver(
      const std::string &_urdf_path, const std::string &_base_link,
      const std::string &_tip_link);  //从urdf构建 并且能够自定义始末
  virtual ~KinematicsSolver() = default;

  bool SolveFK(const Eigen::VectorXf &_joint_positions_in,
               hy_common::geometry::Transform3D &_end_pose_out);

  bool SolveIK(const hy_common::geometry::Transform3D &_end_pose_in,
               Eigen::VectorXf &_joint_positions_out);

  // bool InterpolateTrajectory(const Eigen::VectorXf &_start_joints,
  //                            const Eigen::VectorXf &_end_joints,
  //                            std::vector<KDL::JntArray> &trajectory,
  //                            double duration,
  //                            int num_steps);
  bool InterpolateTrajectory(
      const std::vector<Eigen::VectorXf> &_trajectory_joints,
      JointTrajectory &trajectory, double duration,
      int num_steps);  //原始的轨迹插值 手动设置段时间

  bool InterpolateTrajectory(
      const std::vector<Eigen::VectorXf> &sparse_joints,
      JointTrajectory &trajectory, float max_joint_velocity, float acc_duration,
      float control_frequency);  //新插值 设置关节最大运动速度 五次多项式

  bool InterpolateTrajectory_IPTP(
      const std::vector<Eigen::VectorXf> &sparse_joints,
      JointTrajectory &trajectory, float vel_percentage, float acc_duration,
      float control_frequency);

  bool InterpolateTrajectory_pose(
      const Eigen::VectorXf &current_joints,
      const Eigen::VectorXf &_target_joint_positions,
      JointTrajectory &trajectory, float max_joint_velocity, float acc_duration,
      float control_frequency);  //新插值 梯形速度规划 只对单点pose插值

  bool SamplePose(Eigen::Matrix4f &_sampled_pose_out);

  bool LoadCameraExtrinsics(const CameraExtrinsicParams &_params);

  bool GetCameraExtrinsics(Eigen::Matrix4f &extrinsics_out) const;

  const KDL::Chain &GetChain() const { return chain_; }

  void FitCubicSpline(const int n, const double dt[], const double x[],
                      double x1[], double x2[]);

  void AdjustEndpointAccelerations(const int n, const double dt[], double x[],
                                   double x1[], double x2[]);

  struct SingleJointTrajectory {  // 用于内部计算
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    std::vector<double> jerks;
    double max_velocity, min_velocity;
    double max_acceleration, min_acceleration;
  };
  struct SparseWaypoint {
    Eigen::VectorXf position;
    Eigen::VectorXf velocity;
    Eigen::VectorXf acceleration;
  };

  struct JointLinkInfo {
    std::string joint_name;       // 关节名称
    std::string child_link_name;  // 该段的子link名称
    double max_vel;               // 最大关节速度
    double joint_lower_limit;     // 关节下限
    double joint_upper_limit;     // 关节上限
    double link_length;           // 父link到子link的空间距离
  };

  bool ComputeTimeOptimalProfiles(std::vector<SparseWaypoint> &sparse_waypoints,
                                  std::vector<double> &out_time_diff,
                                  float vel_percentage, float acc_duration);
  bool ExtractLinkJointInfo(const std::string &_urdf_path,
                            std::vector<JointLinkInfo> &infos);

 private:
  bool Initialize();

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;
  KDL::JntArray joint_lower_limits_;
  KDL::JntArray joint_upper_limits_;
  bool has_camera_extrinsics_;
  Eigen::Matrix4f camera_extrinsics_;
  std::unique_ptr<KDL::ChainIdSolver_RNE> id_solver_;
  std::vector<JointControlParams> joint_params_;
  urdf::ModelInterfaceSharedPtr robot_model;
};

}  // namespace hy_manipulation_controllers