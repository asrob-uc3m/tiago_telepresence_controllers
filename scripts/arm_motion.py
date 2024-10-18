#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult
from controller_manager_msgs.srv import ListControllers, SwitchController
from tiago_telepresence_controllers.srv import ArmMotion, ArmMotionResponse

current_state = "home"

def pre_switch_controllers():
  cs = manager_list().controller
  start_controllers = [c.name for c in cs if c.name in controllers and c.state == "stopped"]
  stop_controllers = [c.name for c in cs if c.name.endswith("_tp_controller") and
                                            c.name.replace("_tp_controller", "_controller") in controllers and
                                            c.state == "running"]

  rospy.loginfo("Switching controllers (pre)...")
  rospy.loginfo("Start: %s" % start_controllers)
  rospy.loginfo("Stop: %s" % stop_controllers)

  response = manager_switch(start_controllers=start_controllers,
                            stop_controllers=stop_controllers,
                            strictness=2)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers (pre)")

def post_switch_controllers():
  cs = manager_list().controller
  start_controllers = [c.name for c in cs if c.name.endswith("_tp_controller") and
                                             c.name.replace("_tp_controller", "_controller") in controllers and
                                             c.state == "stopped"]
  stop_controllers = [c.name for c in cs if c.name in controllers and c.state == "running"]

  rospy.loginfo("Switching controllers (post)...")
  rospy.loginfo("Start: %s" % start_controllers)
  rospy.loginfo("Stop: %s" % stop_controllers)

  response = manager_switch(start_controllers=start_controllers,
                            stop_controllers=stop_controllers,
                            strictness=2)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers (post)")

def handle_arm_motion(req):
  global current_state

  if req.command == current_state:
    rospy.loginfo("Motion '%s' already completed." % req.command)
    return ArmMotionResponse(success=True)

  rospy.loginfo("Executing motion: %s", req.command)

  goal = PlayMotionGoal()
  goal.motion_name = req.command
  goal.skip_planning = False

  pre_switch_controllers()
  rospy.sleep(1.0) # important

  client.send_goal(goal)
  client.wait_for_result(rospy.Duration(10.0))
  action_res = client.get_result()

  res = ArmMotionResponse()

  # for some reason, action_res is sometimes None
  if action_res is None or action_res.error_code == PlayMotionResult.SUCCEEDED:
    rospy.loginfo("Motion '%s' completed." % req.command)
    res.success = True
    current_state = req.command
  else:
    rospy.logerr(action_res.error_string)
    res.success = False
    res.message = action_res.error_string

  post_switch_controllers()
  return res

if __name__ == "__main__":
  rospy.init_node("tp_arm_motion")
  rospy.loginfo("Waiting for play_motion...")

  client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
  client.wait_for_server()
  rospy.loginfo("...connected.")

  controllers = rospy.get_param("play_motion/controllers")

  rospy.wait_for_service("controller_manager/list_controllers")
  rospy.wait_for_service("controller_manager/switch_controller")

  manager_list = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
  manager_switch = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)

  s = rospy.Service("tp_arm_motion/command", ArmMotion, handle_arm_motion)

  rospy.spin()