#include "hy_manipulation_controllers/kinematics/kinematics_solver.h"

#include <kdl_parser/kdl_parser.hpp>
#include <random>

namespace hy_manipulation_controllers {

KinematicsSolver::KinematicsSolver(const KDL::Chain &_chain) : chain_(_chain) {
  if (!Initialize()) {
    throw std::runtime_error("Failed to initialize solvers");
  }
  LOG_INFO(
      "[KinematicsSolver] Initialized successfully from existing chain with {} "
      "joints.",
      chain_.getNrOfJoints());
}

KinematicsSolver::KinematicsSolver(const std::string &_urdf_path) {
  urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile(_urdf_path);
  if (!robot_model) {
    throw std::runtime_error("Failed to parse URDF file: " + _urdf_path);
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model, tree)) {
    throw std::runtime_error("Failed to construct KDL tree from URDF model.");
  }
  //寻找base_link和tip_link
  std::string base_link = robot_model->getRoot()->name;
  std::string tip_link;
  for (const auto &link_pair : robot_model->links_) {
    if (link_pair.second->child_joints.empty()) {
      if (tip_link.empty() || link_pair.first.length() > tip_link.length()) {
        tip_link = link_pair.first;  //选最里面的为末端
      }
    }
  }

  if (base_link.empty() || tip_link.empty()) {
    throw std::runtime_error(
        "Failed to automatically determine base or tip link from URDF");
  }

  if (!tree.getChain(base_link, tip_link, chain_)) {
    throw std::runtime_error("Failed to extract KDL chain from tree: " +
                             base_link + " → " + tip_link);
  }

  joint_params_.clear();
  const size_t num_joints = chain_.getNrOfJoints();
  joint_lower_limits_.resize(num_joints);
  joint_upper_limits_.resize(num_joints);

  int joint_idx = 0;
  for (const auto &segment : chain_.segments) {
    const KDL::Joint &joint = segment.getJoint();

    if (joint.getType() == KDL::Joint::None) {
      continue;
    }

    std::string joint_name = joint.getName();
    urdf::JointConstSharedPtr urdf_joint = robot_model->getJoint(joint_name);

    if (!urdf_joint) {
      throw std::runtime_error("Joint '" + joint_name +
                               "' from KDL chain not found in URDF model.");
    }

    JointControlParams params;
    params.id = joint_idx;

    if (urdf_joint->limits) {
      params.max_vel = urdf_joint->limits->velocity;
      params.max_acc = 2.0f * params.max_vel;

      joint_lower_limits_(joint_idx) = urdf_joint->limits->lower;
      joint_upper_limits_(joint_idx) = urdf_joint->limits->upper;

    } else {
      LOG_WARN(
          "Could not find <limit> tag for joint '{}'. Using default limits.",
          joint_name);
      params.max_vel = 1.0f;
      params.max_acc = 2.5f;
      joint_lower_limits_(joint_idx) = -M_PI;
      joint_upper_limits_(joint_idx) = M_PI;
    }

    joint_params_.push_back(params);
    joint_idx++;
  }

  if (!Initialize()) {
    throw std::runtime_error("Failed to initialize solvers.");
  }

  LOG_INFO(
      "[KinematicsSolver] Chain [{} → {}] with {} joints constructed and "
      "limits loaded.",
      base_link, tip_link, num_joints);
}

KinematicsSolver::KinematicsSolver(const std::string &_urdf_path,
                                   const std::string &_base_link,
                                   const std::string &_tip_link) {
  urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile(_urdf_path);
  if (!robot_model) {
    throw std::runtime_error("Failed to parse URDF file: " + _urdf_path);
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model, tree)) {
    throw std::runtime_error("Failed to construct KDL tree from URDF model.");
  }

  if (!tree.getChain(_base_link, _tip_link, chain_)) {
    throw std::runtime_error("Failed to extract KDL chain from tree: " +
                             _base_link + " → " + _tip_link);
  }
  joint_params_.clear();
  const size_t num_joints = chain_.getNrOfJoints();
  joint_lower_limits_.resize(num_joints);
  joint_upper_limits_.resize(num_joints);

  int joint_idx = 0;
  for (const auto &segment : chain_.segments) {
    const KDL::Joint &joint = segment.getJoint();

    if (joint.getType() == KDL::Joint::None) {
      continue;
    }

    std::string joint_name = joint.getName();
    urdf::JointConstSharedPtr urdf_joint = robot_model->getJoint(joint_name);

    if (!urdf_joint) {
      throw std::runtime_error("Joint '" + joint_name +
                               "' from KDL chain not found in URDF model.");
    }

    JointControlParams params;
    params.id = joint_idx;

    if (urdf_joint->limits) {
      params.max_vel = urdf_joint->limits->velocity;
      params.max_acc = 2.0f * params.max_vel;

      joint_lower_limits_(joint_idx) = urdf_joint->limits->lower;
      joint_upper_limits_(joint_idx) = urdf_joint->limits->upper;

    } else {
      LOG_WARN(
          "Could not find <limit> tag for joint '{}'. Using default limits.",
          joint_name);
      params.max_vel = 1.0f;
      params.max_acc = 2.5f;
      joint_lower_limits_(joint_idx) = -M_PI;
      joint_upper_limits_(joint_idx) = M_PI;
    }

    joint_params_.push_back(params);
    joint_idx++;
  }
  if (!Initialize()) {
    throw std::runtime_error(
        "Failed to initialize solvers after loading from URDF");
  }

  LOG_INFO("[KinematicsSolver] Initialized from URDF [{} → {}] with {} joints.",
           _base_link, _tip_link, chain_.getNrOfJoints());
}

bool KinematicsSolver::LoadCameraExtrinsics(
    const CameraExtrinsicParams &_params) {
  Eigen::Matrix3f rotation_matrix;
  rotation_matrix =
      Eigen::AngleAxisf(_params.rotation.z(), Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(_params.rotation.y(), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(_params.rotation.x(), Eigen::Vector3f::UnitX());

  camera_extrinsics_ = Eigen::Matrix4f::Identity();
  camera_extrinsics_.block<3, 3>(0, 0) = rotation_matrix;
  camera_extrinsics_.block<3, 1>(0, 3) = _params.translation;

  has_camera_extrinsics_ = true;

  LOG_INFO("Successfully loaded camera extrinsics for frame {}.",
           _params.child_frame_id);

  return true;
}

bool KinematicsSolver::GetCameraExtrinsics(
    Eigen::Matrix4f &_extrinsics_out) const {
  if (!has_camera_extrinsics_) {
    LOG_ERROR("Camera extrinsics have not been loaded yet.");
    return false;
  }
  _extrinsics_out = camera_extrinsics_;

  return true;
}
bool KinematicsSolver::Initialize() {
  if (chain_.getNrOfJoints() == 0) {
    LOG_ERROR("Initialization failed: KDL chain has zero joints.");
    return false;
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

  if (!fk_solver_) {
    LOG_ERROR("Initialization failed: Could not create FK solver.");
    return false;
  }

  double timeout_in_secs = 0.005;
  double error_tolerance = 1e-5;
  ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
      chain_, joint_lower_limits_, joint_upper_limits_, timeout_in_secs,
      error_tolerance, TRAC_IK::Speed);

  if (!ik_solver_) {
    LOG_ERROR("Initialization failed: Could not create IK solver.");
    return false;
  }

  return true;
}

bool KinematicsSolver::SolveFK(
    const Eigen::VectorXf &_joint_positions_in,
    hy_common::geometry::Transform3D &_end_pose_out) {
  const size_t num_joints = chain_.getNrOfJoints();
  if (_joint_positions_in.size() != num_joints) {
    LOG_ERROR("FK Error: Expected {} joints, but got {}.", num_joints,
              _joint_positions_in.size());
    return false;
  }

  KDL::JntArray q(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    q(i) = _joint_positions_in(i);
  }

  KDL::Frame end_effector_pose;
  int status = fk_solver_->JntToCart(q, end_effector_pose);
  if (status != KDL::ChainFkSolverPos_recursive::E_NOERROR) {
    LOG_ERROR("FK solver failed with status code: {}", status);
    return false;
  }

  // 转换 KDL::Frame 到 Eigen::Matrix4f
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      mat(i, j) = end_effector_pose.M(i, j);
    }
    mat(i, 3) = end_effector_pose.p(i);
  }

  // 使用 Transform3D 构造器创建目标输出
  _end_pose_out = hy_common::geometry::Transform3D(mat);

  return true;
}

bool KinematicsSolver::SolveIK(
    const hy_common::geometry::Transform3D &_end_pose_in,
    Eigen::VectorXf &_joint_positions_out) {
  const size_t num_joints = chain_.getNrOfJoints();

  // 使用零向量作为初始猜值
  KDL::JntArray q_init(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    q_init(i) = 0.0;
  }

  // 将 Transform3D 转为 KDL::Frame
  Eigen::Matrix4f mat = _end_pose_in.GetMatrix4f();
  KDL::Frame end_effector_pose;
  for (int i = 0; i < 3; ++i) {
    end_effector_pose.p(i) = mat(i, 3);
    for (int j = 0; j < 3; ++j) {
      end_effector_pose.M(i, j) = mat(i, j);
    }
  }

  // 解 IK
  KDL::JntArray q_result(num_joints);
  int solution_num = ik_solver_->CartToJnt(q_init, end_effector_pose, q_result);

  if (solution_num >= 0) {
    _joint_positions_out.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
      _joint_positions_out(i) = q_result(i);
    }
    return true;
  }

  LOG_ERROR("IK solver failed to find solution.");
  return false;
}

bool KinematicsSolver::InterpolateTrajectory_pose(
    const Eigen::VectorXf &current_joints,
    const Eigen::VectorXf &_target_joint_positions, JointTrajectory &trajectory,
    float max_joint_velocity, float _acc_duration, float control_frequency) {
  std::vector<float> distances;
  distances.resize(4);
  float max_distance = -1.0;
  int max_distance_joint = -1;
  distances[0] = fabs(current_joints[0] - _target_joint_positions(0));
  for (int i = 1; i < 4; i++) {
    distances[i] = fabs(current_joints[i] - _target_joint_positions(i));
    if (distances[i] > max_distance) {
      max_distance = distances[i];
      max_distance_joint = i;
    }
  }

  if (max_distance_joint == -1) {
    return false;
  }
  float joint0_time_max = fabs(distances[0] / 0.08f);
  float move_time = fabs(max_distance / max_joint_velocity);
  if (max_distance < 0.02 || move_time < joint0_time_max) {
    move_time = joint0_time_max;
    if (move_time < 1e-3) {
      return false;
    }
  }
  LOG_INFO("Move time is [{}],Joint0 time is [{}]", move_time, joint0_time_max);
  if (move_time < 2 * _acc_duration) {
    // 确保总时间至少是两倍的加速时间
    LOG_WARN(
        "Move time is too short for the given acceleration. Adjusting "
        "move_time.");
    move_time = 2 * _acc_duration;
  }

  // 为每个关节创建独立的速度曲线
  std::vector<KDL::VelocityProfile_Trap> profiles;
  for (int i = 0; i < current_joints.size(); ++i) {
    double joint_travel_distance =
        _target_joint_positions(i) - current_joints(i);

    if (move_time <= _acc_duration) {
      LOG_ERROR("move_time must be greater than _acc_duration.");
      return false;
    }

    double max_vel_joint = joint_travel_distance / (move_time - _acc_duration);
    double max_acc_joint = max_vel_joint / _acc_duration;

    KDL::VelocityProfile_Trap profile(fabs(max_vel_joint), fabs(max_acc_joint));
    profile.SetProfile(0, joint_travel_distance);  // 设置起始位置和总位移
    profiles.push_back(profile);
  }

  double time_step = 1.0 / control_frequency;
  size_t num_steps = static_cast<size_t>(move_time / time_step) + 1;
  trajectory.reserve(num_steps * current_joints.size());

  double start_time_sec = ros::Time::now().toSec();  // 获取起始时间戳

  for (double t = 0.0; t <= move_time; t += time_step) {
    double timestamp = start_time_sec + t;
    for (int j = 0; j < current_joints.size(); ++j) {
      JointTrajectoryPoint point;
      point.id = j;
      point.timestamp = timestamp;
      point.position = current_joints(j) + profiles[j].Pos(t);
      point.velocity = profiles[j].Vel(t);
      point.acceleration = profiles[j].Acc(t);

      trajectory.push_back(point);
    }
  }
  return true;
} /*
 调整辅助点的位置，使得加速度能平滑过度
 把辅助点 放在其前和后两个位置 分别生成两段曲线
 用直线近似加速度和位置之间的关系
 */
void KinematicsSolver::AdjustEndpointAccelerations(const int n,
                                                   const double dt[],
                                                   double x[], double x1[],
                                                   double x2[]) {
  float a_init = 0.0f;  //初始加速度
  float a_fina = 0.0f;
  // 存储原始位置作为参考
  const double original_x1 = x[1];
  const double original_xn_2 = x[n - 2];

  // 调整初始加速度
  // 用 x[0] 测试
  x[1] = x[0];
  FitCubicSpline(n, dt, x, x1, x2);
  const double a0 = x2[0];

  // 用 x[2] 测试
  x[1] = x[2];
  FitCubicSpline(n, dt, x, x1, x2);
  const double a2 = x2[0];

  // 求解能产生初加速度的 x[1] 的位置
  if (std::abs(a2 - a0) > 1e-6) {
    x[1] = x[0] + (x[2] - x[0]) * (a_init - a0) / (a2 - a0);
  } else {
    x[1] = original_x1;  // 如果无法调整，恢复原位
  }

  // 调整最终加速度
  // 用 x[n-3] 测试
  x[n - 2] = x[n - 3];
  FitCubicSpline(n, dt, x, x1, x2);
  const double b0 = x2[n - 1];

  // 用 x[n-1] 测试
  x[n - 2] = x[n - 1];
  FitCubicSpline(n, dt, x, x1, x2);
  const double b2 = x2[n - 1];

  // 求解能产生末加速度的 x[n-2] 的位置
  if (std::abs(b2 - b0) > 1e-6) {
    x[n - 2] = x[n - 3] + (x[n - 1] - x[n - 3]) * (a_fina - b0) / (b2 - b0);
  } else {
    x[n - 2] = original_xn_2;  // 如果无法调整，恢复原位
  }
}
void KinematicsSolver::FitCubicSpline(const int n, const double dt[],
                                      const double x[], double x1[],
                                      double x2[]) {
  const double x1_i = x1[0];
  const double x1_f = x1[n - 1];
  std::vector<double> c_tmp(n), d_tmp(n);
  double *c = c_tmp.data();
  double *d = d_tmp.data();
  c[0] = 0.5;
  d[0] = 3.0 * ((x[1] - x[0]) / dt[0] - x1_i) / dt[0];
  for (int i = 1; i <= n - 2; i++) {
    const double dt2 = dt[i - 1] + dt[i];
    const double a = dt[i - 1] / dt2;
    const double denom = 2.0 - a * c[i - 1];
    c[i] = (1.0 - a) / denom;
    d[i] =
        6.0 * ((x[i + 1] - x[i]) / dt[i] - (x[i] - x[i - 1]) / dt[i - 1]) / dt2;
    d[i] = (d[i] - a * d[i - 1]) / denom;
  }
  const double denom = dt[n - 2] * (2.0 - c[n - 2]);
  d[n - 1] = 6.0 * (x1_f - (x[n - 1] - x[n - 2]) / dt[n - 2]);
  d[n - 1] = (d[n - 1] - dt[n - 2] * d[n - 2]) / denom;
  x2[n - 1] = d[n - 1];
  for (int i = n - 2; i >= 0; i--) x2[i] = d[i] - c[i] * x2[i + 1];
  for (int i = 0; i < n - 1; i++)
    x1[i] = (x[i + 1] - x[i]) / dt[i] - (2.0 * x2[i] + x2[i + 1]) * dt[i] / 6.0;
  x1[n - 1] = x1_f;
}

bool KinematicsSolver::ComputeTimeOptimalProfiles(
    std::vector<KinematicsSolver::SparseWaypoint> &sparse_waypoints,
    std::vector<double> &out_time_diff, const float vel_percentage,
    const float acc_duration) {
  const size_t num_points = sparse_waypoints.size();
  const size_t num_joints = sparse_waypoints[0].position.size();

  if (joint_params_.size() != num_joints) {
    LOG_ERROR(
        "Stored joint parameters in KinematicsSolver do not match joint "
        "count.");
    return false;
  }
  for (auto &wp : sparse_waypoints) {
    wp.velocity.resize(num_joints);
    wp.acceleration.resize(num_joints);
  }
  std::vector<double> max_velocities(num_joints);
  std::vector<double> max_accelerations(num_joints);

  for (size_t i = 0; i < num_joints; ++i) {
    double target_velocity = joint_params_[i].max_vel * vel_percentage;
    max_velocities[i] = target_velocity;
    double desired_acceleration = (acc_duration > 1e-4)
                                      ? (target_velocity / acc_duration)
                                      : joint_params_[i].max_acc;  // 防止除以0
    max_accelerations[i] = std::min(
        static_cast<double>(joint_params_[i].max_acc), desired_acceleration);
  }

  // t2 用于储存
  std::vector<SingleJointTrajectory> t2(num_joints);
  for (size_t j = 0; j < num_joints; ++j) {
    t2[j].positions.resize(num_points);
    t2[j].velocities.assign(num_points, 0.0);
    t2[j].accelerations.assign(num_points, 0.0);
    t2[j].jerks.assign(num_points, 0.0);
    for (size_t i = 0; i < num_points; ++i) {
      t2[j].positions[i] = sparse_waypoints[i].position(j);
    }
    t2[j].max_velocity = max_velocities[j];
    t2[j].min_velocity = -max_velocities[j];
    t2[j].max_acceleration = max_accelerations[j];
    t2[j].min_acceleration = -max_accelerations[j];
    LOG_WARN("max_velocities {}  max_acceleration {}", max_velocities[j],
             max_accelerations[j]);
  }

  // 初始化时间参数 并猜测预估时间
  out_time_diff.assign(
      num_points - 1,
      std::numeric_limits<double>::epsilon());  // 用一个很小的正数初始化
  for (size_t j = 0; j < num_joints; ++j) {
    for (size_t i = 0; i < num_points - 1; ++i) {
      double dx = t2[j].positions[i + 1] - t2[j].positions[i];
      double time_needed = (std::abs(dx) > 1e-6)
                               ? std::abs(dx / (dx >= 0 ? t2[j].max_velocity
                                                        : t2[j].min_velocity))
                               : 0.0;
      time_needed += std::numeric_limits<double>::epsilon();  // 防止有零
      if (out_time_diff[i] < time_needed)
        out_time_diff[i] = time_needed;  // 取最大段
    }
  }

  while (1) {
    bool limits_ok = true;
    std::vector<double> time_factor(num_points - 1, 1.0);
    for (size_t j = 0; j < num_joints; ++j) {
      if (num_points >= 4) {
        for (size_t j = 0; j < num_joints; ++j) {
          AdjustEndpointAccelerations(
              num_points, out_time_diff.data(), t2[j].positions.data(),
              t2[j].velocities.data(), t2[j].accelerations.data());
        }
      }

      FitCubicSpline(num_points, out_time_diff.data(), t2[j].positions.data(),
                     t2[j].velocities.data(), t2[j].accelerations.data());
      // 三次
      for (size_t i = 0; i < num_points; ++i) {  // 调整系数 防止加速度超幅
        double acc = t2[j].accelerations[i];
        double atfactor = 1.0;
        if (acc > t2[j].max_acceleration)
          atfactor = sqrt(acc / t2[j].max_acceleration);
        else if (acc < t2[j].min_acceleration)
          atfactor = sqrt(acc / t2[j].min_acceleration);
        if (atfactor > 1.001) {
          limits_ok = false;
          atfactor = (atfactor - 1.0) / 16.0 + 1.0;
          if (i > 0)
            time_factor[i - 1] = std::max(time_factor[i - 1], atfactor);
          if (i < num_points - 1)
            time_factor[i] = std::max(time_factor[i], atfactor);
        }
      }
    }
    if (limits_ok) break;
    for (size_t i = 0; i < num_points - 1; ++i)
      out_time_diff[i] *= time_factor[i];
  }

  // 全局调整 确保没有超过边界
  double gtfactor = 1.0;
  for (size_t j = 0; j < num_joints; ++j) {
    FitCubicSpline(num_points, out_time_diff.data(), t2[j].positions.data(),
                   t2[j].velocities.data(), t2[j].accelerations.data());
    for (size_t i = 0; i < num_points; ++i) {
      double v_factor = 1.0, a_factor = 1.0;
      if (t2[j].velocities[i] > t2[j].max_velocity)
        v_factor = t2[j].velocities[i] / t2[j].max_velocity;
      if (t2[j].velocities[i] < t2[j].min_velocity)
        v_factor = t2[j].velocities[i] / t2[j].min_velocity;
      if (t2[j].accelerations[i] > t2[j].max_acceleration)
        a_factor = sqrt(t2[j].accelerations[i] / t2[j].max_acceleration);
      if (t2[j].accelerations[i] < t2[j].min_acceleration)
        a_factor = sqrt(t2[j].accelerations[i] / t2[j].min_acceleration);
      gtfactor = std::max({gtfactor, v_factor, a_factor});
    }
  }

  if (gtfactor > 1.0) {
    for (size_t i = 0; i < num_points - 1; ++i) out_time_diff[i] *= gtfactor;
  }

  // 将最终计算结果从 t2 转置回 sparse_waypoints
  for (size_t j = 0; j < num_joints; ++j) {
    FitCubicSpline(num_points, out_time_diff.data(), t2[j].positions.data(),
                   t2[j].velocities.data(), t2[j].accelerations.data());

    for (size_t i = 0; i < num_points; ++i) {
      sparse_waypoints[i].position(j) = t2[j].positions[i];
      sparse_waypoints[i].velocity(j) = t2[j].velocities[i];
      sparse_waypoints[i].acceleration(j) = t2[j].accelerations[i];
    }
  }

  return true;
}
bool KinematicsSolver::InterpolateTrajectory_IPTP(
    const std::vector<Eigen::VectorXf> &sparse_joints,
    JointTrajectory &trajectory, float vel_percentage, float acc_duration,
    float control_frequency) {
  if (sparse_joints.size() < 2) {
    LOG_ERROR("Trajectory must have at least two waypoints.");
    return false;
  }
  trajectory.clear();
  const size_t num_joints = sparse_joints[0].size();

  std::vector<SparseWaypoint> waypoints_to_process;
  for (const auto &p : sparse_joints) {
    waypoints_to_process.push_back({p, Eigen::VectorXf(), Eigen::VectorXf()});
  }
  // 首尾添加辅助点
  if (waypoints_to_process.size() >= 2) {
    SparseWaypoint start_helper;
    start_helper.position = (9.0 * waypoints_to_process[0].position +
                             1.0 * waypoints_to_process[1].position) /
                            10.0;
    waypoints_to_process.insert(waypoints_to_process.begin() + 1, start_helper);

    SparseWaypoint end_helper;
    end_helper.position =
        (1.0 * waypoints_to_process[waypoints_to_process.size() - 2].position +
         9.0 * waypoints_to_process.back().position) /
        10.0;
    waypoints_to_process.insert(waypoints_to_process.end() - 1, end_helper);
  }

  std::vector<double> time_diff;
  //用三次样条插值得到中间点的速度信息
  if (!ComputeTimeOptimalProfiles(waypoints_to_process, time_diff,
                                  vel_percentage, acc_duration)) {
    LOG_ERROR("Failed to compute time-optimal profiles.");
    return false;
  }
  //以下都是测试部分
  LOG_INFO("--- DEBUG: Computed Profiles for {} waypoints: ---",
           waypoints_to_process.size());
  for (size_t i = 0; i < waypoints_to_process.size(); ++i) {
    // 使用 stringstream 来格式化 Eigen 向量的输出
    std::stringstream ss_pos, ss_vel, ss_acc;
    ss_pos << waypoints_to_process[i].position.transpose();
    ss_vel << waypoints_to_process[i].velocity.transpose();
    ss_acc << waypoints_to_process[i].acceleration.transpose();

    LOG_INFO("  Waypoint {}:", i);
    if (i > 0) {
      LOG_INFO("    TimeDiff from prev: {}", time_diff[i - 1]);
    }
    LOG_INFO("    Pos: [ {} ]", ss_pos.str().c_str());
    LOG_INFO("    Vel: [ {} ]", ss_vel.str().c_str());
    LOG_INFO("    Acc: [ {} ]", ss_acc.str().c_str());
  }
  LOG_INFO("--- END DEBUG ---");

  const size_t num_points = waypoints_to_process.size();
  const double dt = 1.0 / control_frequency;
  double total_duration =
      std::accumulate(time_diff.begin(), time_diff.end(), 0.0);

  if (total_duration < 1e-3) {
    LOG_WARN("Total trajectory duration is near zero. No motion needed.");
    return true;
  }

  std::vector<KDL::VelocityProfile_Spline> splines(num_joints);
  const double start_time_sec = ros::Time::now().toSec();
  double current_time_offset = 0.0;

  trajectory.reserve(static_cast<size_t>(total_duration * control_frequency) +
                     num_points);

  for (size_t i = 0; i < num_points - 1; ++i) {
    const double seg_duration = time_diff[i];
    if (seg_duration < 1e-6) continue;

    for (size_t j = 0; j < num_joints; ++j) {
      splines[j].SetProfileDuration(waypoints_to_process[i].position(j),
                                    waypoints_to_process[i].velocity(j),
                                    waypoints_to_process[i].acceleration(j),
                                    waypoints_to_process[i + 1].position(j),
                                    waypoints_to_process[i + 1].velocity(j),
                                    waypoints_to_process[i + 1].acceleration(j),
                                    seg_duration);
    }

    const int num_steps_in_segment =
        static_cast<int>(std::round(seg_duration / dt));
    for (int k = 0; k < num_steps_in_segment; ++k) {
      const double t_local = k * dt;
      for (size_t j = 0; j < num_joints; ++j) {
        JointTrajectoryPoint point;
        point.id = j;
        point.timestamp = start_time_sec + current_time_offset + t_local;
        point.position = splines[j].Pos(t_local);
        point.velocity = splines[j].Vel(t_local);
        point.acceleration = splines[j].Acc(t_local);
        trajectory.push_back(point);
      }
    }
    current_time_offset += seg_duration;
  }

  for (size_t j = 0; j < num_joints; ++j) {
    JointTrajectoryPoint final_point;
    final_point.id = j;
    final_point.timestamp = start_time_sec + total_duration;
    final_point.position = waypoints_to_process.back().position(j);
    final_point.velocity = 0.0;
    final_point.acceleration = 0.0;
    trajectory.push_back(final_point);
  }

  LOG_INFO(
      "Successfully interpolated trajectory with {} points over {} seconds.",
      trajectory.size() / num_joints, total_duration);

  return true;
}
bool KinematicsSolver::InterpolateTrajectory(
    const std::vector<Eigen::VectorXf> &sparse_joints,
    JointTrajectory &trajectory, float max_joint_velocity, float acc_duration,
    float control_frequency) {
  if (sparse_joints.size() < 2) {
    LOG_ERROR("Trajectory must have at least two waypoints.");
    return false;
  }
  if (max_joint_velocity <= 0.0f || control_frequency <= 0.0f) {
    LOG_ERROR("Max joint velocity and control frequency must be positive.");
    return false;
  }

  trajectory.clear();
  const size_t num_waypoints = sparse_joints.size();
  const size_t num_joints = sparse_joints[0].size();
  for (const auto &jnt : sparse_joints) {
    if (jnt.size() != num_joints) {
      LOG_ERROR("All waypoints must have the same number of joints.");
      return false;
    }
  }

  // 动态计算每段轨迹的持续时间
  std::vector<double> segment_durations;
  double total_duration = 0.0;
  for (size_t i = 0; i < num_waypoints - 1; ++i) {
    const Eigen::VectorXf &q_start = sparse_joints[i];
    const Eigen::VectorXf &q_end = sparse_joints[i + 1];

    double max_angular_distance = 0.0;
    for (size_t j = 0; j < num_joints; ++j) {
      double angular_distance = std::abs(q_end(j) - q_start(j));
      if (angular_distance > max_angular_distance) {
        max_angular_distance = angular_distance;
      }
    }
    double duration_0 = std::abs(q_end(0) - q_start(0)) /
                        0.06f;  // 移动关节最大速度达到所需要时间
    double duration = 0.0;
    if (max_angular_distance > 0.005f) {
      duration = max_angular_distance / max_joint_velocity;
    }
    if (duration < duration_0) {
      duration = duration_0;  // 取最大的时间
    }
    segment_durations.push_back(duration);
    total_duration += duration;
  }

  if (total_duration < 1e-3) {
    LOG_WARN("Total trajectory duration is near zero. No motion needed.");
    return true;
  }

  // 边界条件
  std::vector<std::vector<double>> positions(
      num_joints, std::vector<double>(num_waypoints));
  std::vector<std::vector<double>> velocities(
      num_joints, std::vector<double>(num_waypoints, 0.0));
  std::vector<std::vector<double>> accelerations(
      num_joints, std::vector<double>(num_waypoints, 0.0));

  for (size_t i = 0; i < num_waypoints; ++i) {
    for (size_t j = 0; j < num_joints; ++j) {
      positions[j][i] = sparse_joints[i](j);
    }
  }

  for (size_t j = 0; j < num_joints; ++j) {
    velocities[j][0] = 0.0;
    accelerations[j][0] = 0.0;
    velocities[j][num_waypoints - 1] = 0.0;
    accelerations[j][num_waypoints - 1] = 0.0;

    for (size_t i = 1; i < num_waypoints - 1; ++i) {
      double h1 = segment_durations[i - 1];
      double h2 = segment_durations[i];

      if (h1 < 1e-6 || h2 < 1e-6) {
        velocities[j][i] = 0.0;
        accelerations[j][i] = 0.0;
        continue;
      }

      double p_prev = positions[j][i - 1];
      double p_curr = positions[j][i];
      double p_next = positions[j][i + 1];

      velocities[j][i] = (p_next - p_curr) / h2 * (h1 / (h1 + h2)) +
                         (p_curr - p_prev) / h1 * (h2 / (h1 + h2));
      accelerations[j][i] =
          2.0 * ((p_next - p_curr) / h2 - (p_curr - p_prev) / h1) / (h1 + h2);
    }
  }

  // 密集轨迹
  const double dt = 1.0 / control_frequency;
  double current_time_offset = 0.0;
  const double start_time_sec = ros::Time::now().toSec();

  std::vector<KDL::VelocityProfile_Spline> splines(num_joints);
  trajectory.reserve(static_cast<size_t>(total_duration * control_frequency) *
                         num_joints +
                     num_joints);

  for (size_t i = 0; i < num_waypoints - 1; ++i) {
    const double seg_duration = segment_durations[i];
    if (seg_duration < 1e-6) {
      continue;
    }

    for (size_t j = 0; j < num_joints; ++j) {
      splines[j].SetProfileDuration(positions[j][i], velocities[j][i],
                                    accelerations[j][i], positions[j][i + 1],
                                    velocities[j][i + 1],
                                    accelerations[j][i + 1], seg_duration);
    }

    const int num_steps_in_segment =
        static_cast<int>(std::round(seg_duration / dt));
    for (int k = 0; k < num_steps_in_segment; ++k) {
      const double t_local = k * dt;
      for (size_t j = 0; j < num_joints; ++j) {
        JointTrajectoryPoint point;
        point.id = j;
        point.timestamp = start_time_sec + current_time_offset + t_local;
        point.position = splines[j].Pos(t_local);
        point.velocity = splines[j].Vel(t_local);
        point.acceleration = splines[j].Acc(t_local);
        trajectory.push_back(point);
      }
    }
    current_time_offset += seg_duration;
  }

  for (size_t j = 0; j < num_joints; ++j) {
    JointTrajectoryPoint final_point;
    final_point.id = j;
    final_point.timestamp = start_time_sec + total_duration;
    final_point.position = positions[j][num_waypoints - 1];
    final_point.velocity = 0.0;
    final_point.acceleration = 0.0;
    trajectory.push_back(final_point);
  }

  LOG_INFO(
      "Successfully interpolated trajectory with {} points over {:.2f} "
      "seconds.",
      trajectory.size() / num_joints, total_duration);

  return true;
}
bool KinematicsSolver::InterpolateTrajectory(
    const std::vector<Eigen::VectorXf> &_trajectory_joints,
    JointTrajectory &trajectory, double duration, int num_steps) {
  if (_trajectory_joints.size() < 2) {
    LOG_ERROR("Trajectory must have at least two waypoints.");
    return false;
  }
  if (duration <= 0.0) {
    LOG_ERROR("Duration must be positive.");
    return false;
  }
  if (num_steps < 2) {
    LOG_ERROR("Number of steps must be at least 2.");
    return false;
  }
  const size_t num_waypoints = _trajectory_joints.size();
  const size_t num_joints = _trajectory_joints[0].size();
  for (const auto &jnt : _trajectory_joints) {
    if (jnt.size() != num_joints) {
      LOG_ERROR("All waypoints must have the same number of joints.");
      return false;
    }
  }

  trajectory.clear();

  trajectory.reserve(num_steps * num_joints);

  std::vector<std::vector<double>> joint_points(
      num_joints, std::vector<double>(num_waypoints));
  for (size_t i = 0; i < num_waypoints; ++i) {
    for (size_t j = 0; j < num_joints; ++j) {
      joint_points[j][i] = _trajectory_joints[i](j);
    }
  }

  double segment_duration = duration / (num_waypoints - 1);
  if (segment_duration <= 1e-6) {
    LOG_ERROR(
        "Segment duration is too small. Check total duration and number of "
        "waypoints.");
    return false;
  }

  std::vector<std::vector<double>> velocities(
      num_joints, std::vector<double>(num_waypoints, 0.0));
  std::vector<std::vector<double>> accelerations(
      num_joints, std::vector<double>(num_waypoints, 0.0));

  for (size_t j = 0; j < num_joints; ++j) {
    // 轨迹的起点和终点，速度和加速度都为零
    velocities[j][0] = 0.0;
    velocities[j][num_waypoints - 1] = 0.0;
    accelerations[j][0] = 0.0;
    accelerations[j][num_waypoints - 1] = 0.0;

    // 中间路径点的速度和加速度  中间差速 确保连续
    for (size_t i = 1; i < num_waypoints - 1; ++i) {
      double p_prev = joint_points[j][i - 1];
      double p_curr = joint_points[j][i];
      double p_next = joint_points[j][i + 1];

      // v(i) ≈ (p(i+1) - p(i-1)) / (2*h)
      velocities[j][i] = (p_next - p_prev) / (2.0 * segment_duration);

      // a(i) ≈ (p(i+1) - 2*p(i) + p(i-1)) / h^2
      accelerations[j][i] = (p_next - 2.0 * p_curr + p_prev) /
                            (segment_duration * segment_duration);
    }
  }

  double start_time_sec = ros::Time::now().toSec();
  std::vector<KDL::VelocityProfile_Spline> splines(num_joints);
  double dt = duration / (num_steps - 1);
  int last_seg_idx = -1;

  for (int i = 0; i < num_steps; ++i) {
    double t = i * dt;
    int seg_idx = (t >= duration) ? (num_waypoints - 2)
                                  : static_cast<int>(t / segment_duration);
    seg_idx = std::max(0, std::min(seg_idx, (int)num_waypoints - 2));

    if (seg_idx != last_seg_idx) {
      for (size_t j = 0; j < num_joints; ++j) {
        // 获取当前段的起点和终点边界条件
        double p0 = joint_points[j][seg_idx];
        double v0 = velocities[j][seg_idx];
        double a0 = accelerations[j][seg_idx];

        double p1 = joint_points[j][seg_idx + 1];
        double v1 = velocities[j][seg_idx + 1];
        double a1 = accelerations[j][seg_idx + 1];

        // 使用精确的边界条件来配置样条曲线
        splines[j].SetProfileDuration(p0, v0, a0, p1, v1, a1, segment_duration);
      }
      last_seg_idx = seg_idx;
    }

    double t_local = t - seg_idx * segment_duration;
    double real_timestamp = start_time_sec + t;

    for (size_t j = 0; j < num_joints; ++j) {
      JointTrajectoryPoint point;
      point.id = j;
      point.timestamp = real_timestamp;
      point.position = splines[j].Pos(t_local);
      point.velocity = splines[j].Vel(t_local);
      point.acceleration = splines[j].Acc(t_local);
      trajectory.push_back(point);
    }
  }
  LOG_INFO("num_waypoints:{},segment_duration:{}", num_waypoints,
           segment_duration);
  if (trajectory.size() >= num_joints) {
    const auto &target_point_vec = _trajectory_joints.back();
    for (size_t j = 0; j < num_joints; ++j) {
      JointTrajectoryPoint &last_point =
          trajectory[trajectory.size() - num_joints + j];
      last_point.position = target_point_vec(j);
      last_point.velocity = 0.0f;
      last_point.acceleration = 0.0f;
    }
  }

  return true;
}

bool KinematicsSolver::SamplePose(Eigen::Matrix4f &_sampled_pose_out) {
  // const size_t num_joints = chain_.getNrOfJoints();
  // Eigen::VectorXf random_joints(num_joints);

  // std::random_device rd;
  // std::mt19937 gen(rd());

  // for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i)
  // {
  //     std::uniform_real_distribution<> dis(joint_lower_limits_(i),
  //                                          joint_upper_limits_(i));
  //     random_joints(i) = dis(gen);
  // }

  // return SolveFK(random_joints, _sampled_pose_out);
  return true;
}

}  // namespace hy_manipulation_controllers