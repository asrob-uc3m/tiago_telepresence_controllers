#include "controller_base.hpp"

#include <algorithm> // std::max, std::min
#include <urdf/model.h>
#include <std_msgs/Float32MultiArray.h>

using namespace tiago_controllers;

constexpr auto INPUT_TIMEOUT = 0.25; // [s]

bool ControllerBase::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    ROS_INFO("Initializing %s controller", name.c_str());

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

    if (isStepping && !n.getParam("step", step))
    {
        ROS_ERROR("Could not retrieve step");
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

    jointAngles.resize(joints.size());
    std::string out;

    for (int i = 0; i < joints.size(); i++)
    {
        jointAngles[i] = joints[i].getPosition();
        out += " " + std::to_string(jointAngles[i]);
    }

    ROS_INFO("Initial joint configuration for %s:%s", name.c_str(), out.c_str());

    return additionalSetup(hw, n);
}

void ControllerBase::registerPublisher(ros::NodeHandle &n, ros::Publisher &pub)
{
    pub = n.advertise<std_msgs::Float32MultiArray>("state", 1);
}

void ControllerBase::updateStamp()
{
    // callers must lock mutex
    stamp = ros::Time::now();
}

ros::Time ControllerBase::getLastStamp() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return stamp;
}

void ControllerBase::update(const ros::Time& time, const ros::Duration& period)
{
    static const ros::Duration timeout(INPUT_TIMEOUT);

    std_msgs::Float32MultiArray msg;

    for (const auto & joint : joints)
    {
        msg.data.push_back(joint.getPosition());
    }

    pub.publish(msg);

    bool wasActive = isActive;

    if (time - getLastStamp() > timeout)
    {
        isActive = false;
        return;
    }

    if (!wasActive)
    {
        onStarting();
    }

    isActive = true;

    const auto targets = getDesiredJointValues(period);

    if (targets.size() == joints.size())
    {
        for (int i = 0; i < joints.size(); i++)
        {
            const auto & limits = jointLimits[i];
            const auto target = isStepping ? (msg.data[i] + step * targets[i]) : targets[i];

            jointAngles[i] = std::max(limits.first, std::min(limits.second, target));
            joints[i].setCommand(jointAngles[i]);
        }
    }
}
