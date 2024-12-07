#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>

namespace tiago_controllers
{

class TorsoController : public StepperGenericController<std_msgs::Int32>
{
public:
    TorsoController() : StepperGenericController("torso") { }

protected:
    void processData(const std_msgs::Int32& msg) override
    {
        accept({static_cast<double>(msg.data)});
    }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::TorsoController, controller_interface::ControllerBase);
