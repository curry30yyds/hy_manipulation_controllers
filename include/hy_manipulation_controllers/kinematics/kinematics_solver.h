#pragma once

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <memory>
#include <queue>
#include <trac_ik/trac_ik.hpp>
#include <random>

#include "hy_common/geometry/core/transform.h"
#include <hy_common/logger/logger.h>
#include "hy_manipulation_controllers/utils/config.h"
// #include "hy_manipulation_controllers/core/arm_controller.h"

namespace hy_manipulation_controllers
{

  class KinematicsSolver
  {
  public:
    typedef std::shared_ptr<KinematicsSolver> Ptr;

  public:
    KinematicsSolver(const KDL::Chain &_chain);
    KinematicsSolver(const std::string &_urdf_path);
    KinematicsSolver(const std::string &_urdf_path,
                     const std::string &_base_link,
                     const std::string &_tip_link);
    virtual ~KinematicsSolver() = default;

    bool SolveFK(const Eigen::VectorXf &_joint_positions_in,
                 Eigen::Matrix4f &_end_pose_out);

    bool SolveIK(const Eigen::Matrix4f &_end_pose_in,
                 Eigen::VectorXf &_joint_positions_in,
                 Eigen::VectorXf &_joint_positions_out);

    bool InterpolateTrajectory(const Eigen::VectorXf &_start_joints,
                               const Eigen::VectorXf &_end_joints,
                               std::vector<KDL::JntArray> &trajectory,
                               double duration,
                               int num_steps);

    bool SamplePose(Eigen::Matrix4f &_sampled_pose_out);

    void SetCameraExtrinsics(const KDL::Frame &_camera_to_base_transform);

    KDL::Frame TransformPoseFromCameraToBase(
        const KDL::Frame &_pose_in_camera) const;

    const KDL::Chain &GetChain() const { return chain_; }

  private:
    bool Initialize();

    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;
    KDL::JntArray joint_lower_limits_;
    KDL::JntArray joint_upper_limits_;
    KDL::Frame camera_to_base_transform_;
  };

} // namespace hy_manipulation_controllers