#pragma once

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <memory>
#include <queue>
#include <trac_ik/trac_ik.hpp>

#include "hy_common/geometry/core/transform.h"

namespace hy_manipulation_controllers {

class KinematicsSolver {
 public:
  typedef std::shared_ptr<KinematicsSolver> Ptr;

 public:
  KinematicsSolver(const KDL::Chain& _chain);
  KinematicsSolver(const std::string& _urdf_path);

  bool SolveFK();

  bool SolveIK();

  bool InterpolateTrajectory();
};

}  // namespace hy_manipulation_controllers