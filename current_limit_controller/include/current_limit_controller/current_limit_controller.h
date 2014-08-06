///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, PAL Robotics S.L.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PAL_ROS_CONTROLLERS_CURRENT_LIMIT_CONTROLLER_H
#define PAL_ROS_CONTROLLERS_CURRENT_LIMIT_CONTROLLER_H

#include <vector>
#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <pal_ros_control/current_limit_interface.h>

namespace pal_control_msgs
{
ROS_DECLARE_MESSAGE(ActuatorCurrentLimit);
}

namespace pal_ros_controllers
{

/** \brief Controller that allows to set the current limit for a group of actuators. */
class CurrentLimitController : public controller_interface::Controller<pal_ros_control::CurrentLimitInterface>
{
public:
  CurrentLimitController() : controller_interface::Controller<pal_ros_control::CurrentLimitInterface>() {}

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(pal_ros_control::CurrentLimitInterface* hw, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

private:
  std::string ctrl_name_; ///< Controller name.
  std::vector<std::string> names_; ///< Names of actuators used by this controller.
  realtime_tools::RealtimeBuffer<std::vector<double> > cmd_; ///< Ordered as the list of actuator names.
  std::vector<double> null_cmd_;
  std::vector<pal_ros_control::CurrentLimitHandle> handles_; ///< Handles to controlled actuators.
  ros::Subscriber curr_lim_sub_;

  void commandCB(const pal_control_msgs::ActuatorCurrentLimitConstPtr& msg);
};

} // namespace

#endif // Header guard
