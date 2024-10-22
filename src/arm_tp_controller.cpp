#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

constexpr auto UPDATE_LOG_THROTTLE = 1.0; // [s]
constexpr auto TIMEOUT = 1.0; // [s]

namespace
{
    KDL::JntArray vectorToKdl(const std::vector<double> & q)
    {
        KDL::JntArray ret(q.size());

        for (int i = 0; i < q.size(); i++)
        {
            ret(i) = q[i];
        }

        return ret;
    }

    std::vector<double> kdlToVector(const KDL::JntArray & q)
    {
        // https://stackoverflow.com/a/26094702
        const auto & priv = q.data;
        return std::vector<double>(priv.data(), priv.data() + priv.rows());
    }
}

namespace tiago_controllers
{

class ArmController : public GenericController<geometry_msgs::Pose>
{
public:
    ArmController() : GenericController("arm", false) { }

    ~ArmController()
    {
        delete fkSolverPos;
        delete ikSolverVel;
    }

    void onStarting(const std::vector<double> & angles) override
    {
        q = vectorToKdl(angles);
        fkSolverPos->JntToCart(q, H_0_N_initial);
        ROS_INFO("Initial position: %f %f %f", H_0_N_initial.p.x(), H_0_N_initial.p.y(), H_0_N_initial.p.z());
        H_0_N_prev = H_0_N_initial;
    }

protected:
    bool additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) override;
    bool getDesiredJointValues(const ros::Duration& period, const std::vector<double> & current, std::vector<double> & desired) override;

private:
    bool checkReturnCode(int ret);

    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive * fkSolverPos {nullptr};
    KDL::ChainIkSolverVel_pinv * ikSolverVel {nullptr};
    KDL::Frame H_0_N_initial;
    KDL::Frame H_0_N_prev;
    KDL::JntArray q;
};

} // namespace tiago_controllers

bool tiago_controllers::ArmController::additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(robot_desc_string, tree))
    {
        ROS_ERROR("Failed to construct KDL tree");
        return false;
    }

    std::string start_link, end_link;

    if (!n.getParam("start_link", start_link))
    {
        ROS_ERROR("Could not find start_link parameter");
        return false;
    }

    if (!n.getParam("end_link", end_link))
    {
        ROS_ERROR("Could not find end_link parameter");
        return false;
    }

    if (!tree.getChain(start_link, end_link, chain))
    {
        ROS_ERROR("Failed to get chain from kdl tree");
        return false;
    }

    ROS_INFO("Got chain with %d joints and %d segments", chain.getNrOfJoints(), chain.getNrOfSegments());

    double eps;
    int maxIter;

    if (!n.getParam("ik_solver_vel_eps", eps))
    {
        ROS_ERROR("Could not find ik_solver_vel_eps parameter");
        return false;
    }

    if (!n.getParam("ik_solver_vel_max_iter", maxIter))
    {
        ROS_ERROR("Could not find ik_solver_vel_max_iter parameter");
        return false;
    }

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
    ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, eps, maxIter);

    return true;
}

bool tiago_controllers::ArmController::getDesiredJointValues(const ros::Duration& period, const std::vector<double> & current, std::vector<double> & desired)
{
    KDL::Frame H_N; // change in pose between initial and desired, referred to the TCP

    {
        std::lock_guard<std::mutex> lock(mutex);
        H_N.p = KDL::Vector(value.position.x, value.position.y, value.position.z);
        H_N.M = KDL::Rotation::Quaternion(value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w);
    }

    auto H_0_N_desired = H_0_N_initial * H_N;
    auto twist = KDL::diff(H_0_N_prev, H_0_N_desired, period.toSec());

    // refer to base frame, but leave the reference point intact
    twist = H_0_N_initial.M * twist;

    KDL::JntArray qdot(getJointCount());

    if (!checkReturnCode(ikSolverVel->CartToJnt(vectorToKdl(current), twist, qdot)))
    {
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "Could not calculate joint velocities (1)");
        return false;
    }

    const auto & limits = getJointLimits();
    auto q_temp = q;

    for (int i = 0; i < getJointCount(); i++)
    {
        q_temp(i) += qdot(i) * period.toSec();

        if (q_temp(i) < limits[i].first || q_temp(i) > limits[i].second)
        {
            ROS_WARN("Joint %d out of limits: %f not in [%f, %f]", i, q_temp(i), limits[i].first, limits[i].second);
            return false;
        }
    }

    KDL::JntArray qdot_temp(chain.getNrOfJoints());

    if (!checkReturnCode(ikSolverVel->CartToJnt(q_temp, twist, qdot_temp)))
    {
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "Could not calculate joint velocities (2)");
        return false;
    }

    // no singular point, so update the calculated pose and joint values
    H_0_N_prev = H_0_N_desired;
    q = q_temp;

    desired = kdlToVector(q);
    return true;
}

bool tiago_controllers::ArmController::checkReturnCode(int ret)
{
    switch (ret)
    {
    case KDL::ChainIkSolverVel_pinv::E_CONVERGE_PINV_SINGULAR:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "Convergence issue: pseudo-inverse is singular");
        return false;
    case KDL::SolverI::E_SVD_FAILED:
        ROS_ERROR_THROTTLE(UPDATE_LOG_THROTTLE, "Convergence issue: SVD failed");
        return false;
    case KDL::SolverI::E_NOERROR:
        return true;
    default:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "Convergence issue: unknown error");
        return false;
    }
}

PLUGINLIB_EXPORT_CLASS(tiago_controllers::ArmController, controller_interface::ControllerBase);
