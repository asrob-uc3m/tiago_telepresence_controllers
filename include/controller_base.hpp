#ifndef __TIAGO_TP_CONTROLLER_BASE_HPP__
#define __TIAGO_TP_CONTROLLER_BASE_HPP__

#include <mutex>
#include <string>
#include <utility> // std::pair
#include <vector>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace tiago_controllers
{

class ControllerBase : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
    ControllerBase(const std::string &_name, bool _isStepping) : name(_name), isStepping(_isStepping) {}
    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) final override;
    void update(const ros::Time& time, const ros::Duration& period) final override;

protected:
    std::string getName() const { return name; }
    int getJointCount() const { return joints.size(); }
    const std::vector<std::pair<double, double>> & getJointLimits() const { return jointLimits; }
    virtual bool additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) { return true; }
    virtual void registerSubscriber(ros::NodeHandle &n, ros::Subscriber &sub) = 0;
    virtual std::vector<double> getDesiredJointValues(const ros::Duration& period) = 0;
    void updateStamp();
    const std::vector<double> & getJoints() const { return jointAngles; }
    virtual void onStarting() {}

    std::string robot_desc_string;
    mutable std::mutex mutex;

private:
    void registerPublisher(ros::NodeHandle &n, ros::Publisher &pub);
    ros::Time getLastStamp() const;

    ros::Subscriber sub;
    ros::Publisher pub;
    std::string name;
    bool isStepping {false};
    bool isActive {false};
    std::vector<hardware_interface::JointHandle> joints;
    std::vector<std::pair<double, double>> jointLimits;
    std::vector<double> jointAngles;
    double step {0.0};
    ros::Time stamp;
};

} // namespace tiago_controllers

#endif // __TIAGO_TP_CONTROLLER_BASE_HPP__
