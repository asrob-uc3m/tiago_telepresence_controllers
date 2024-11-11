#ifndef __TIAGO_GENERIC_TP_CONTROLLER_HPP__
#define __TIAGO_GENERIC_TP_CONTROLLER_HPP__

#include <mutex>

#include "controller_base.hpp"
#include "command_buffer.hpp"

namespace tiago_controllers
{

template <typename T>
class GenericController : public ControllerBase
{
public:
    using ControllerBase::ControllerBase;

protected:
    void registerSubscriber(ros::NodeHandle &n, ros::Subscriber &sub) final override
    {
        sub = n.subscribe<T>("command", 1, &GenericController::callback, this);
    }

    virtual void processData(const T& msg) = 0;
    virtual void accept(const std::vector<double> & command) = 0;

private:
    void callback(const typename T::ConstPtr& msg)
    {
        updateStamp();
        processData(*msg);
    }
};

template <typename T>
class BufferedGenericController : public GenericController<T>
{
public:
    using GenericController<T>::GenericController;

protected:
    void onStarting(const std::vector<double> & angles) override
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        buffer.reset(angles);
    }

    void accept(const std::vector<double> & command) override final
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        buffer.accept(command);
    }

    double getCommandPeriod() const
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        const auto period = buffer.getCommandPeriod();
        return period.toSec();
    }

private:
    std::vector<double> getDesiredJointValues(const std::vector<double> & current) override final
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        return buffer.interpolate();
    }

    CommandBuffer buffer;
    mutable std::mutex bufferMutex;
};

template <typename T>
class StepperGenericController : public GenericController<T>
{
public:
    using GenericController<T>::GenericController;

protected:
    bool additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n, const std::string &description) override
    {
        if (!n.getParam("step", step) || step <= 0.0)
        {
            ROS_ERROR("Could not retrieve step or not greater than 0");
            return false;
        }

        return true;
    }

    void accept(const std::vector<double> & command) override final
    {
        std::lock_guard<std::mutex> lock(storageMutex);
        stored = command;
    }

private:
    std::vector<double> getDesiredJointValues(const std::vector<double> & current) override final
    {
        std::vector<double> desired(current.size());
        std::lock_guard<std::mutex> lock(storageMutex);

        for (int i = 0; i < current.size(); i++)
        {
            desired[i] = current[i] + step * stored[i];
        }

        return desired;
    }

    double step {0.0};
    std::vector<double> stored;
    mutable std::mutex storageMutex;
};

} // namespace tiago_controllers

#endif // __TIAGO_GENERIC_TP_CONTROLLER_HPP__
