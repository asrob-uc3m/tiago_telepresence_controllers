#include "controller_base.hpp"

#include <algorithm> // std::max, std::min
#include <urdf/model.h>
#include <std_msgs/Float32MultiArray.h>

using namespace tiago_controllers;

constexpr auto INPUT_TIMEOUT = 0.25; // [s]

bool ControllerBase::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    ROS_INFO("Initializing %s controller", name.c_str());

    std::string robot_desc_string;

    if (!n.getParam("/robot_description", robot_desc_string))
    {
        ROS_ERROR("Could not find robot_description");
        return false;
    }

    urdf::Model model;

    if (!model.initString(robot_desc_string))
    {
        ROS_ERROR("Failed to parse URDF file");
        return false;
    }

    std::vector<std::string> joint_names;

    if (!n.getParam("joint_names", joint_names))
    {
        ROS_ERROR("Could not retrieve joint names");
        return false;
    }

    for (const auto & joint_name : joint_names)
    {
        const auto & joint = model.getJoint(joint_name);

        if (!joint)
        {
            ROS_ERROR("Could not retrieve joint %s", joint_name.c_str());
            return false;
        }

        joints.push_back(hw->getHandle(joint_name));
        jointLimits.emplace_back(joint->limits->lower, joint->limits->upper);
    }

    registerSubscriber(n, sub);
    registerPublisher(n, pub);

    std::vector<double> initial(joints.size());
    std::string out;

    for (int i = 0; i < joints.size(); i++)
    {
        initial[i] = joints[i].getPosition();
        out += " " + std::to_string(initial[i]);
    }

    ROS_INFO("Initial joint configuration for %s:%s", name.c_str(), out.c_str());

    return additionalSetup(hw, n, robot_desc_string);
}

void ControllerBase::registerPublisher(ros::NodeHandle &n, ros::Publisher &pub)
{
    pub = n.advertise<std_msgs::Float32MultiArray>("state", 1);
}

void ControllerBase::updateStamp()
{
    std::lock_guard<std::mutex> lock(stampMutex);
    stamp = ros::Time::now();
}

ros::Time ControllerBase::getLastStamp() const
{
    std::lock_guard<std::mutex> lock(stampMutex);
    return stamp;
}

void ControllerBase::update(const ros::Time& time, const ros::Duration& period)
{
    static const ros::Duration timeout(INPUT_TIMEOUT);

    std_msgs::Float32MultiArray msg;
    std::vector<double> current;

    for (const auto & joint : joints)
    {
        const auto position = joint.getPosition();
        msg.data.push_back(position);
        current.push_back(position);
    }

    //pub.publish(msg);

    if (time - getLastStamp() > timeout)
    {
        if (isActive)
        {
            isActive = false;
            onDisabling();
        }

        return;
    }
    else if (!isActive)
    {
        isActive = true;
        onStarting(current);
    }

    const auto desired = getDesiredJointValues(current);

    for (int i = 0; i < joints.size(); i++)
    {
        const auto & limits = jointLimits[i];
        const auto cmd = std::max(limits.first, std::min(limits.second, desired[i]));

        joints[i].setCommand(cmd);
    }
}
