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
                                   Eigen::Matrix4f &_end_pose_out)
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

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                _end_pose_out(i, j) = end_effector_pose.M(i, j);
            }
            _end_pose_out(i, 3) = end_effector_pose.p(i);
        }

        _end_pose_out.row(3) << 0, 0, 0, 1;

        return status == KDL::ChainFkSolverPos_recursive::E_NOERROR;
    }

    bool KinematicsSolver::SolveIK(const Eigen::Matrix4f &_end_pose_in,
                                   Eigen::VectorXf &_joint_positions_in,
                                   Eigen::VectorXf &_joint_positions_out)
    {
        const size_t num_joints = chain_.getNrOfJoints();

        if (_joint_positions_in.size() != num_joints)
        {
            LOG_ERROR("IK Error: Current joint state size ({}) doesn't match chain DOF ({}).", _joint_positions_in.size(), num_joints);
            return false;
        }

        KDL::JntArray q_init(num_joints);
        for (size_t i = 0; i < num_joints; ++i)
        {
            q_init(i) = _joint_positions_in(i);
        }

        KDL::Frame end_effector_pose;
        for (int i = 0; i < 3; ++i)
        {
            end_effector_pose.p(i) = _end_pose_in(i, 3);
            for (int j = 0; j < 3; ++j)
            {
                end_effector_pose.M(i, j) = _end_pose_in(i, j);
            }
        }

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

        return false;
    }

    bool KinematicsSolver::InterpolateTrajectory(const Eigen::VectorXf &_start_joints,
                                                 const Eigen::VectorXf &_end_joints,
                                                 std::vector<KDL::JntArray> &trajectory,
                                                 double duration,
                                                 int num_steps)
    {
        trajectory.clear();

        const size_t dof = _start_joints.size();
        if (_end_joints.size() != dof || num_steps < 2)
        {
            LOG_ERROR("Invalid input for trajectory interpolation.");
            return false;
        }

        trajectory.reserve(num_steps);

        // 五次多项式轨迹系数：每个关节一个 coeff[6]
        std::vector<std::array<double, 6>> coeffs(dof);

        double T = duration;
        double T2 = T * T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;

        // 初始速度/加速度设为 0
        double v0 = 0.0, a0 = 0.0;
        double vf = 0.0, af = 0.0;

        for (size_t i = 0; i < dof; ++i)
        {
            double p0 = _start_joints(i);
            double pf = _end_joints(i);

            auto &c = coeffs[i];
            c[0] = p0;
            c[1] = v0;
            c[2] = 0.5 * a0;
            c[3] = (20 * (pf - p0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T2) / (2.0 * T3);
            c[4] = (-30 * (pf - p0) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T2) / (2.0 * T4);
            c[5] = (12 * (pf - p0) - (6 * vf + 6 * v0) * T - (a0 - af) * T2) / (2.0 * T5);
        }

        // 时间步长
        for (int step = 0; step < num_steps; ++step)
        {
            double t = (duration * step) / (num_steps - 1);
            double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

            KDL::JntArray q(dof);
            for (size_t i = 0; i < dof; ++i)
            {
                auto &c = coeffs[i];
                q(i) = c[0] + c[1] * t + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;
            }

            trajectory.push_back(q);
        }

        return true;
    }

    bool KinematicsSolver::SamplePose(Eigen::Matrix4f &_sampled_pose_out)
    {
        const size_t num_joints = chain_.getNrOfJoints();
        Eigen::VectorXf random_joints(num_joints);

        std::random_device rd;
        std::mt19937 gen(rd());

        for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i)
        {
            std::uniform_real_distribution<> dis(joint_lower_limits_(i),
                                                 joint_upper_limits_(i));
            random_joints(i) = dis(gen);
        }

        return SolveFK(random_joints, _sampled_pose_out);
    }

    void KinematicsSolver::SetCameraExtrinsics(
        const KDL::Frame &_camera_to_base_transform)
    {
        camera_to_base_transform_ = _camera_to_base_transform;
    }

    KDL::Frame KinematicsSolver::TransformPoseFromCameraToBase(
        const KDL::Frame &_pose_in_camera) const
    {
        return camera_to_base_transform_ * _pose_in_camera;
    }

} // namespace hy_manipulation_controllers