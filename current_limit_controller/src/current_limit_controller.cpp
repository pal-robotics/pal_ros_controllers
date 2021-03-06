///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, PAL Robotics S.L.
//
///////////////////////////////////////////////////////////////////////////////

// C++ standard
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <current_limit_controller/current_limit_controller.h>


namespace pal_ros_controllers
{

namespace internal
{
std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

/**
 * \return The permutation vector between two containers.
 * If \p t1 is <tt>"{A, B, C, D}"</tt> and \p t2 is <tt>"{B, D, A, C}"</tt>, the associated permutation vector is
 * <tt>"{2, 0, 3, 1}"</tt>.
 */
template <class T>
inline std::vector<unsigned int> permutation(const T& t1, const T& t2)
{
  typedef unsigned int SizeType;

  // Arguments must have the same size
  if (t1.size() != t2.size()) {return std::vector<SizeType>();}

  std::vector<SizeType> permutation_vector(t1.size()); // Return value
  for (typename T::const_iterator t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    typename T::const_iterator t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {return std::vector<SizeType>();}
    else
    {
      const SizeType t1_dist = std::distance(t1.begin(), t1_it);
      const SizeType t2_dist = std::distance(t2.begin(), t2_it);
      permutation_vector[t1_dist] = t2_dist;
    }
  }
  return permutation_vector;
}

} // namespace

bool CurrentLimitController::init(pal_ros_control::CurrentLimitInterface* hw, ros::NodeHandle& controller_nh)
{
  // Controller name
  ctrl_name_ = internal::getLeafNamespace(controller_nh);

  // Resource names
  const std::string param_name = "actuators";
  const bool names_ok = controller_nh.getParam(param_name, names_);
  if (!names_ok)
  {
    ROS_ERROR_STREAM_NAMED(ctrl_name_, "Could not find '" << param_name << "' parameter (namespace: " <<
                                       controller_nh.getNamespace() << ").");
    return false;
  }

  // State publish rate
  double state_pub_rate = 50.0;
  controller_nh.getParam("state_publish_rate", state_pub_rate);
  ROS_DEBUG_STREAM_NAMED(ctrl_name_, "Controller state will be published at " << state_pub_rate << "Hz.");
  state_pub_period_ = ros::Duration(1.0 / state_pub_rate);

  // Resource handles
  for (unsigned int i = 0; i < names_.size(); ++i)
  {
    try
    {
      pal_ros_control::CurrentLimitHandle handle = hw->getHandle(names_[i]);
      handles_.push_back(handle);
    }
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(ctrl_name_, "Could not find actuator '" << names_[i] << "' in '" <<
                                         this->getHardwareInterfaceType() << "'.");
      return false;
    }
  }

  // Initialize null command value
  null_cmd_ = std::vector<double>(names_.size(), std::numeric_limits<double>::quiet_NaN());

  // ROS API: Subscribed topics
  curr_lim_sub_ = controller_nh.subscribe("command", 1, &CurrentLimitController::commandCB, this);

  // ROS API: Published topics
  state_pub_.reset(new StatePublisher(controller_nh, "state", 1));

  // Preeallocate resources
  {
    state_pub_->lock();
    state_pub_->msg_.actuator_names = names_;
    state_pub_->msg_.current_limits.resize(names_.size());
    state_pub_->unlock();
  }

  return true;
}


void CurrentLimitController::starting(const ros::Time& time)
{
  // Set null command
  cmd_.writeFromNonRT(null_cmd_);

  // Initialize last state update time
  last_state_pub_time_ = time;
}

void CurrentLimitController::stopping(const ros::Time& time)
{
}

void CurrentLimitController::update(const ros::Time& time, const ros::Duration& period)
{
  std::vector<double>* new_cmd_ptr = cmd_.readFromRT();
  const std::vector<double>& new_cmd = *new_cmd_ptr;
  assert(new_cmd.size() == handles_.size());

  for (unsigned int i = 0; i < handles_.size(); ++i)
  {
    handles_[i].setCurrentLimit(new_cmd[i]);
  }

  publishState(time);
}

void CurrentLimitController::commandCB(const pal_control_msgs::ActuatorCurrentLimitConstPtr& msg)
{
  // Preconditions
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(ctrl_name_, "Can't accept new commands. Controller is not running.");
    return;
  }

  if (!msg)
  {
    ROS_WARN_NAMED(ctrl_name_, "Received null-pointer trajectory message, skipping.");
    return;
  }

  std::vector<unsigned int> permutation = internal::permutation(names_, msg->actuator_names);
  if (permutation.empty())
  {
    ROS_ERROR("Cannot set current limits. Message does not contain the expected actuators.");
    return;
  }
  assert(permutation.size() == names_.size());

  // Set new command
  std::vector<double> cmd_new;
  cmd_new.reserve(names_.size());

  for (unsigned int i = 0; i < msg->current_limits.size(); ++i)
  {
    const unsigned int id = permutation[i];
    cmd_new.push_back(msg->current_limits[id]);
  }
  cmd_.writeFromNonRT(cmd_new);
}

void CurrentLimitController::publishState(const ros::Time& time)
{
  // Check if it's time to publish
  if (!state_pub_period_.isZero() && last_state_pub_time_ + state_pub_period_ < time)
  {
    if (state_pub_ && state_pub_->trylock())
    {
      last_state_pub_time_ += state_pub_period_;

      for (unsigned int i = 0; i < handles_.size(); ++i)
      {
        const double val = handles_[i].getCurrentLimit();
        state_pub_->msg_.current_limits[i] = std::isnan(val) ? 1.0 : val; // TODO: Fix in actuators manager side
      }

      state_pub_->unlockAndPublish();
    }
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS(pal_ros_controllers::CurrentLimitController, controller_interface::ControllerBase)
