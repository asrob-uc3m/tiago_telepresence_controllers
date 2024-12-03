#include "generic_tp_controller.hpp"

#include <atomic>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

constexpr auto UPDATE_LOG_THROTTLE = 1.0; // [s]

namespace tiago_controllers
{

class ArmController : public FrameBufferController<geometry_msgs::PoseStamped>
{
public:
    ArmController() : FrameBufferController("arm") { }

    ~ArmController() override
    {
        delete ikSolverPos;
        delete fkSolverPos;
        delete ikSolverVel;
    }

    void onStarting(const std::vector<double> & angles) override
    {
        q_slow = q_fast = jointVectorToKdl(angles);
        fkSolverPos->JntToCart(q_slow, H_0_N_initial);
        ROS_INFO("[%s] Initial position: %f %f %f", getName().c_str(), H_0_N_initial.p.x(), H_0_N_initial.p.y(), H_0_N_initial.p.z());
        H_0_N_prev_slow = H_0_N_prev_fast = H_0_N_initial;
        FrameBufferController::onStarting(angles);
        active = true;
    }

    void onDisabling() override
    {
        active = false;
    }

protected:
    bool additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n, const std::string &description) override;
    void processData(const geometry_msgs::PoseStamped& msg) override;
    KDL::Frame convertToBufferType(const std::vector<double> & v) override;
    std::vector<double> convertToVector(const KDL::Frame & H_0_N, double period) override;

private:
    bool checkLimits(const KDL::JntArray & q);
    bool checkReturnCode(int ret);

    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive * fkSolverPos {nullptr};
    KDL::ChainIkSolverPos_NR * ikSolverPos {nullptr};
    KDL::ChainIkSolverVel_pinv * ikSolverVel {nullptr};

    KDL::Frame H_0_N_initial;
    KDL::Frame H_0_N_prev_slow;
    KDL::Frame H_0_N_prev_fast;

    KDL::JntArray qMin;
    KDL::JntArray qMax;

    KDL::JntArray q_slow;
    KDL::JntArray q_fast;

    std::atomic_bool active {false};
};

} // namespace tiago_controllers

using namespace tiago_controllers;

bool ArmController::additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n, const std::string &description)
{
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(description, tree))
    {
        ROS_ERROR("[%s] Failed to construct KDL tree", getName().c_str());
        return false;
    }

    std::string start_link, end_link;

    if (!n.getParam("start_link", start_link))
    {
        ROS_ERROR("[%s] Could not find start_link parameter", getName().c_str());
        return false;
    }

    if (!n.getParam("end_link", end_link))
    {
        ROS_ERROR("[%s] Could not find end_link parameter", getName().c_str());
        return false;
    }

    if (!tree.getChain(start_link, end_link, chain))
    {
        ROS_ERROR("[%s] Failed to get chain from kdl tree", getName().c_str());
        return false;
    }

    ROS_INFO("[%s] Got chain with %d joints and %d segments", getName().c_str(), chain.getNrOfJoints(), chain.getNrOfSegments());

    double epsPos, epsVel;
    int maxIterPos, maxIterVel;

    if (!n.getParam("ik_solver_pos_eps", epsPos))
    {
        ROS_ERROR("[%s] Could not find ik_solver_pos_eps parameter", getName().c_str());
        return false;
    }

    if (!n.getParam("ik_solver_pos_max_iter", maxIterPos))
    {
        ROS_ERROR("[%s] Could not find ik_solver_pos_max_iter parameter", getName().c_str());
        return false;
    }

    if (!n.getParam("ik_solver_vel_eps", epsVel))
    {
        ROS_ERROR("[%s] Could not find ik_solver_vel_eps parameter", getName().c_str());
        return false;
    }

    if (!n.getParam("ik_solver_vel_max_iter", maxIterVel))
    {
        ROS_ERROR("[%s] Could not find ik_solver_vel_max_iter parameter", getName().c_str());
        return false;
    }

    const auto & limits = getJointLimits();

    qMin.resize(limits.size());
    qMax.resize(limits.size());

    for (auto i = 0; i < limits.size(); i++)
    {
        qMin(i) = limits[i].first;
        qMax(i) = limits[i].second;
    }

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
    ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, epsVel, maxIterVel);
    ikSolverPos = new KDL::ChainIkSolverPos_NR(chain, *fkSolverPos, *ikSolverVel, epsPos, maxIterPos);

    return FrameBufferController::additionalSetup(hw, n, description);
}

void ArmController::processData(const geometry_msgs::PoseStamped& msg)
{
    if (active)
    {
        // change in pose between initial and desired, referred to the TCP on its initial pose
        KDL::Frame H_N(
            KDL::Rotation::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
            KDL::Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        );

        auto H_0_N = H_0_N_initial * H_N;
        // KDL::JntArray qd(q_slow.rows());

        // if (ikSolverPos->CartToJnt(q_slow, H_0_N, qd) >= 0 && checkLimits(qd))
        // {
        //     H_0_N_prev_fast = H_0_N;
        //     q_slow = qd;
        // }
        // else
        // {
        //     ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Could not solve IK", getName().c_str());
        // }

        // accept(H_0_N_prev_fast, msg.header.stamp);
        accept(H_0_N, msg.header.stamp);
    }
}

KDL::Frame ArmController::convertToBufferType(const std::vector<double> & v)
{
    const auto q = jointVectorToKdl(v);
    KDL::Frame H;
    fkSolverPos->JntToCart(q, H);
    return H;
}

std::vector<double> ArmController::convertToVector(const KDL::Frame & H_0_N, double period)
{
    auto twist = KDL::diff(H_0_N_prev_slow, H_0_N, period);

    // refer to base frame, but leave the reference point intact
    twist = H_0_N_prev_slow.M * twist;

    KDL::JntArray qdot(q_fast.rows());

    if (checkReturnCode(ikSolverVel->CartToJnt(q_fast, twist, qdot)))
    {
        for (int i = 0; i < q_fast.rows(); i++)
        {
            q_fast(i) += qdot(i) * period;
        }

        H_0_N_prev_slow = H_0_N;
    }

    return kdlToJointVector(q_fast);
}

bool ArmController::checkLimits(const KDL::JntArray & q)
{
    for (int i = 0; i < q.rows(); i++)
    {
        if (q(i) < qMin(i) || q(i) > qMax(i))
        {
            ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Joint %d out of limits: %f not in [%f, %f]",
                              getName().c_str(), i, q(i), qMin(i), qMax(i));
            return false;
        }
    }

    return true;
}

bool ArmController::checkReturnCode(int ret)
{
    switch (ret)
    {
    case KDL::ChainIkSolverVel_pinv::E_CONVERGE_PINV_SINGULAR:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Convergence issue: pseudo-inverse is singular", getName().c_str());
        return false;
    case KDL::SolverI::E_SVD_FAILED:
        ROS_ERROR_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Convergence issue: SVD failed", getName().c_str());
        return false;
    case KDL::SolverI::E_NOERROR:
        return true;
    default:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Convergence issue: unknown error", getName().c_str());
        return false;
    }
}

// don't remove the `tiago_controllers` namespace here
PLUGINLIB_EXPORT_CLASS(tiago_controllers::ArmController, controller_interface::ControllerBase);
