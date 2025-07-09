#include "hy_manipulation_controllers/kinematics/kinematics_solver.h"

#include <kdl_parser/kdl_parser.hpp>
#include <random>

namespace hy_manipulation_controllers
{

    KinematicsSolver::KinematicsSolver(const KDL::Chain &_chain) : chain_(_chain)
    {
        if (!Initialize())
        {
            throw std::runtime_error("Failed to initialize solvers");
        }
        LOG_INFO("[KinematicsSolver] Initialized successfully from existing chain with {} joints.",
                 chain_.getNrOfJoints());
    }

    KinematicsSolver::KinematicsSolver(const std::string &_urdf_path)
    {
        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(_urdf_path, tree))
        {
            throw std::runtime_error("Failed to construct KDL tree from URDF file: " + _urdf_path);
        }

        // 自动寻找 base_link 和 tip_link
        std::string base_link, tip_link;

        // base_link: tree的根节点
        base_link = tree.getRootSegment()->first;

        // tip_link: 找到最深的子节点作为末端（也可以更复杂地指定）
        size_t max_depth = 0;
        for (const auto &segment : tree.getSegments())
        {
            const std::string &link_name = segment.first;

            // tip_link 应该是叶子节点（无子节点）
            if (segment.second.children.empty())
            {
                // 简单策略：取名字最长的（最深层）
                if (link_name.length() > tip_link.length())
                {
                    tip_link = link_name;
                }
            }
        }

        if (base_link.empty() || tip_link.empty())
        {
            throw std::runtime_error("Failed to automatically determine base or tip link from URDF");
        }

        if (!tree.getChain(base_link, tip_link, chain_))
        {
            throw std::runtime_error("Failed to extract KDL chain from tree: " +
                                     base_link + " → " + tip_link);
        }

        if (!Initialize())
        {
            throw std::runtime_error("Failed to initialize solvers after loading from URDF");
        }

        LOG_INFO("[KinematicsSolver] Auto-inferred chain from URDF [{} → {}] with {} joints.",
                 base_link, tip_link, chain_.getNrOfJoints());
    }

    KinematicsSolver::KinematicsSolver(const std::string &_urdf_path,
                                       const std::string &_base_link,
                                       const std::string &_tip_link)
    {
        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(_urdf_path, tree))
        {
            throw std::runtime_error("Failed to construct KDL tree from URDF file: " + _urdf_path);
        }

        if (!tree.getChain(_base_link, _tip_link, chain_))
        {
            throw std::runtime_error("Failed to extract KDL chain from tree: " +
                                     _base_link + " → " + _tip_link);
        }

        if (!Initialize())
        {
            throw std::runtime_error("Failed to initialize solvers after loading from URDF");
        }

        LOG_INFO("[KinematicsSolver] Initialized from URDF [{} → {}] with {} joints.",
                 _base_link, _tip_link, chain_.getNrOfJoints());
    }

    bool KinematicsSolver::LoadCameraExtrinsics(const CameraExtrinsicParams &_params)
    {

        Eigen::Matrix3f rotation_matrix;
        rotation_matrix = Eigen::AngleAxisf(_params.rotation.z(), Eigen::Vector3f::UnitZ()) *
                          Eigen::AngleAxisf(_params.rotation.y(), Eigen::Vector3f::UnitY()) *
                          Eigen::AngleAxisf(_params.rotation.x(), Eigen::Vector3f::UnitX());

        camera_extrinsics_ = Eigen::Matrix4f::Identity();
        camera_extrinsics_.block<3, 3>(0, 0) = rotation_matrix;
        camera_extrinsics_.block<3, 1>(0, 3) = _params.translation;

        has_camera_extrinsics_ = true;

        LOG_INFO("Successfully loaded camera extrinsics for frame {}.", _params.child_frame_id);

        return true;
    }

    bool KinematicsSolver::GetCameraExtrinsics(Eigen::Matrix4f &_extrinsics_out) const
    {
        if (!has_camera_extrinsics_)
        {
            LOG_ERROR("Camera extrinsics have not been loaded yet.");
            return false;
        }
        _extrinsics_out = camera_extrinsics_;

        return true;
    }
    bool KinematicsSolver::Initialize()
    {

        if (chain_.getNrOfJoints() == 0)
        {
            LOG_ERROR("Initialization failed: KDL chain has zero joints.");
            return false;
        }

        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

        if (!fk_solver_)
        {
            LOG_ERROR("Initialization failed: Could not create FK solver.");
            return false;
        }

        unsigned int num_joints = chain_.getNrOfJoints();
        joint_lower_limits_.resize(num_joints);
        joint_upper_limits_.resize(num_joints);
        for (unsigned int i = 0; i < num_joints; ++i)
        {
            joint_lower_limits_(i) = -M_PI;
            joint_upper_limits_(i) = M_PI;
        }

        double timeout_in_secs = 0.005;
        double error_tolerance = 1e-5;
        ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
            chain_, joint_lower_limits_, joint_upper_limits_, timeout_in_secs,
            error_tolerance, TRAC_IK::Speed);

        if (!ik_solver_)
        {
            LOG_ERROR("Initialization failed: Could not create IK solver.");
            return false;
        }

        return true;
    }

    bool KinematicsSolver::SolveFK(const Eigen::VectorXf &_joint_positions_in,
                                   hy_common::geometry::Transform3D &_end_pose_out)
    {
        const size_t num_joints = chain_.getNrOfJoints();
        if (_joint_positions_in.size() != num_joints)
        {
            LOG_ERROR("FK Error: Expected {} joints, but got {}.", num_joints, _joint_positions_in.size());
            return false;
        }

        KDL::JntArray q(num_joints);
        for (size_t i = 0; i < num_joints; ++i)
        {
            q(i) = _joint_positions_in(i);
        }

        KDL::Frame end_effector_pose;
        int status = fk_solver_->JntToCart(q, end_effector_pose);
        if (status != KDL::ChainFkSolverPos_recursive::E_NOERROR)
        {
            LOG_ERROR("FK solver failed with status code: {}", status);
            return false;
        }

        // 转换 KDL::Frame 到 Eigen::Matrix4f
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                mat(i, j) = end_effector_pose.M(i, j);
            }
            mat(i, 3) = end_effector_pose.p(i);
        }

        // 使用 Transform3D 构造器创建目标输出
        _end_pose_out = hy_common::geometry::Transform3D(mat);

        return true;
    }

    bool KinematicsSolver::SolveIK(const hy_common::geometry::Transform3D &_end_pose_in,
                                   Eigen::VectorXf &_joint_positions_out)
    {
        const size_t num_joints = chain_.getNrOfJoints();

        // 使用零向量作为初始猜值（也可改为上一次解的值）
        KDL::JntArray q_init(num_joints);
        for (size_t i = 0; i < num_joints; ++i)
        {
            q_init(i) = 0.0;
        }

        // 将 Transform3D 转为 KDL::Frame
        Eigen::Matrix4f mat = _end_pose_in.GetMatrix4f();
        KDL::Frame end_effector_pose;
        for (int i = 0; i < 3; ++i)
        {
            end_effector_pose.p(i) = mat(i, 3);
            for (int j = 0; j < 3; ++j)
            {
                end_effector_pose.M(i, j) = mat(i, j);
            }
        }

        // 解 IK
        KDL::JntArray q_result(num_joints);
        int solution_num = ik_solver_->CartToJnt(q_init, end_effector_pose, q_result);

        if (solution_num >= 0)
        {
            _joint_positions_out.resize(num_joints);
            for (size_t i = 0; i < num_joints; ++i)
            {
                _joint_positions_out(i) = q_result(i);
            }
            return true;
        }

        LOG_ERROR("IK solver failed to find solution.");
        return false;
    }

    bool KinematicsSolver::InterpolateTrajectory(const std::vector<Eigen::VectorXf> &_trajectory_joints,
                                                 JointTrajectory &trajectory,
                                                 double duration,
                                                 int num_steps)
    {
        if (_trajectory_joints.size() < 2)
        {
            LOG_ERROR("Trajectory must have at least two waypoints.");
            return false;
        }
        if (duration <= 0.0)
        {
            LOG_ERROR("Duration must be positive.");
            return false;
        }
        if (num_steps < 2)
        {
            LOG_ERROR("Number of steps must be at least 2.");
            return false;
        }
        const size_t num_waypoints = _trajectory_joints.size();
        const size_t num_joints = _trajectory_joints[0].size();
        for (const auto &jnt : _trajectory_joints)
        {
            if (jnt.size() != num_joints)
            {
                LOG_ERROR("All waypoints must have the same number of joints.");
                return false;
            }
        }

        trajectory.clear();
        trajectory.reserve(num_steps);

        std::vector<std::vector<double>> joint_points(num_joints, std::vector<double>(num_waypoints));
        for (size_t i = 0; i < num_waypoints; ++i)
        {
            for (size_t j = 0; j < num_joints; ++j)
            {
                joint_points[j][i] = _trajectory_joints[i](j);
            }
        }

        double segment_duration = duration / (num_waypoints - 1);
        std::vector<std::vector<double>> velocities(num_joints, std::vector<double>(num_waypoints, 0.0));

        for (size_t j = 0; j < num_joints; ++j)
        {
            // 起始点和结束点速度为0
            velocities[j][0] = 0.0;
            velocities[j][num_waypoints - 1] = 0.0;

            // 计算中间路径点的速度  待定
            for (size_t i = 1; i < num_waypoints - 1; ++i)
            {
                // double p_prev = joint_points[j][i - 1];
                // double p_next = joint_points[j][i + 1];
                // velocities[j][i] = (p_next - p_prev) / (2.0 * segment_duration);
                velocities[j][i] = 0.0;
            }
        }
        double start_time_sec = ros::Time::now().toSec();
        std::vector<KDL::VelocityProfile_Spline> splines(num_joints);
        double dt = duration / (num_steps - 1);
        int last_seg_idx = -1;

        for (int i = 0; i < num_steps; ++i)
        {
            double t = i * dt;

            // 确定当前时间 t 所在的轨迹段索引
            int seg_idx = (t >= duration) ? (num_waypoints - 2) : static_cast<int>(t / segment_duration);

            // 如果进入了一个新的段，则为新段重新配置样条曲线
            if (seg_idx != last_seg_idx)
            {
                for (size_t j = 0; j < num_joints; ++j)
                {
                    double p0 = joint_points[j][seg_idx];
                    double v0 = velocities[j][seg_idx];
                    double p1 = joint_points[j][seg_idx + 1];
                    double v1 = velocities[j][seg_idx + 1];

                    // 起始和结束加速度都设为0
                    splines[j].SetProfileDuration(p0, v0, 0.0, p1, v1, 0.0, segment_duration);
                }
                last_seg_idx = seg_idx;
            }

            double t_local = t - seg_idx * segment_duration;
            double real_timestamp = start_time_sec + t;

            for (size_t j = 0; j < num_joints; ++j)
            {
                JointTrajectoryPoint point;
                point.id = j;
                point.timestamp = real_timestamp;
                point.position = splines[j].Pos(t_local);
                point.velocity = splines[j].Vel(t_local);
                point.acceleration = splines[j].Acc(t_local);
                trajectory.push_back(point);
            }
        }

        if (trajectory.size() >= num_joints)
        {
            const auto &target_point_vec = _trajectory_joints.back();
            for (size_t j = 0; j < num_joints; ++j)
            {
                // 定位到最后一个时间戳的对应关节
                JointTrajectoryPoint &last_point = trajectory[trajectory.size() - num_joints + j];
                last_point.position = target_point_vec(j);
                last_point.velocity = 0.0f;
                last_point.acceleration = 0.0f;
            }
        }

        return true;
    }

    bool KinematicsSolver::SamplePose(Eigen::Matrix4f &_sampled_pose_out)
    {
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

} // namespace hy_manipulation_controllers