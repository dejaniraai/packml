/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2017 Shaun Edwards
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "packml_ros/packml_ros.h"
#include "packml_msgs/utils.h"

namespace packml_ros
{


PackmlRos::PackmlRos(ros::NodeHandle nh, ros::NodeHandle pn,
          std::shared_ptr<packml_sm::StateMachine> sm) :
 nh_(nh), pn_(pn), sm_(sm)
{
  ros::NodeHandle packml_node("~/packml");

  status_pub_ = packml_node.advertise<packml_msgs::Status>("status", 10, true);
  trans_server_ = packml_node.advertiseService("transition", &PackmlRos::transRequest, this);
  status_msg_ = packml_msgs::initStatus(pn.getNamespace());

  //Added to be compatible with our Python code (START)
  start_button_ = nh_.advertise<std_msgs::Bool>("start_button", 1);
  abort_button_ = nh_.advertise<std_msgs::Bool>("abort_button", 1);
  clear_button_ = nh_.advertise<std_msgs::Bool>("clear_button", 1);
  hold_button_ = nh_.advertise<std_msgs::Bool>("hold_button", 1);
  reset_button_ = nh_.advertise<std_msgs::Bool>("reset_button", 1);
  unsuspend_button_ = nh_.advertise<std_msgs::Bool>("unsuspend_button", 1);
  unhold_button_ = nh_.advertise<std_msgs::Bool>("unhold_button", 1);
  suspend_button_ = nh_.advertise<std_msgs::Bool>("suspend_button", 1);
  stop_button_ = nh_.advertise<std_msgs::Bool>("stop_button", 1);
  //Added to be compatible with our Python code (END)

  connect(sm.get(), SIGNAL(stateChanged(int, QString)), this, SLOT(pubState(int, QString)));
}

void PackmlRos::spin()
{
  while(ros::ok())
  {
    spinOnce();
  }
  return;
}


void PackmlRos::spinOnce()
{
  ros::spinOnce();
  QCoreApplication::instance()->processEvents();
}


void PackmlRos::pubState(int value, QString name)
{
  ROS_DEBUG_STREAM("Publishing state change: " << name.toStdString() << "(" << value << ")");

  status_msg_.header.stamp = ros::Time().now();
  if( packml_msgs::isStandardState(value) )
  {
    status_msg_.state.val = value;
    status_msg_.sub_state = packml_msgs::State::UNDEFINED;
  }
  else
  {
    status_msg_.sub_state = value;
  }
  status_pub_.publish(status_msg_);

}

bool PackmlRos::transRequest(packml_msgs::Transition::Request &req,
                  packml_msgs::Transition::Response &res)
{
  bool command_rtn = false;
  bool command_valid = true;
  int command_int = static_cast<int>(req.command);
  std::stringstream ss;

  ROS_DEBUG_STREAM("Evaluating transition request command: " << command_int);

  switch(command_int) {
  case req.ABORT:
  case req.ESTOP:
    command_rtn = sm_->abort();
    //Added to be compatible with our Python code (START)
    abort_button_.publish(true);
    ros::Duration(0.5).sleep();
    abort_button_.publish(false);
    //Added to be compatible with our Python code (END)
    break;
  case req.CLEAR:
    command_rtn = sm_->clear();
    //Added to be compatible with our Python code (START)
    clear_button_.publish(true);
    ros::Duration(0.5).sleep();
    clear_button_.publish(false);
    //Added to be compatible with our Python code (END)
    break;
  case req.HOLD:
    command_rtn = sm_->hold();
    //Added to be compatible with our Python code (START)
    hold_button_.publish(true);
    ros::Duration(0.5).sleep();
    hold_button_.publish(false);
    //Added to be compatible with our Python code (END)
    break;
  case req.RESET:
    command_rtn = sm_->reset();
    //Added to be compatible with our Python code (START)
    reset_button_.publish(true);
    ros::Duration(0.5).sleep();
    reset_button_.publish(false);
    //Added to be compatible with our Python code (END)
    break;
  case req.START:
    command_rtn = sm_->start();
    //Added to be compatible with our Python code (START)
    start_button_.publish(true);
    ros::Duration(0.5).sleep();
    start_button_.publish(false);
    //Added to be compatible with our Python code (END)
    break;
  case req.STOP:
    command_rtn = sm_->stop();
    //Added to be compatible with our Python code (START)
    stop_button_.publish(true);
    ros::Duration(0.5).sleep();
    stop_button_.publish(false);
    //Added to be compatible with our Python code (END)
    break;
  case req.SUSPEND:
    command_rtn = sm_->suspend();
    //Added to be compatible with our Python code (START)
    suspend_button_.publish(true);
    ros::Duration(0.5).sleep();
    suspend_button_.publish(false);
    //Added to be compatible with our Python code (END)
    break;
  case req.UNHOLD:
    command_rtn = sm_->unhold();
    //Added to be compatible with our Python code (START)
    unhold_button_.publish(true);
    ros::Duration(0.5).sleep();
    unhold_button_.publish(false);
    //Added to be compatible with our Python code (END)
    break;
  case req.UNSUSPEND:
    command_rtn = sm_->unsuspend();
    //Added to be compatible with our Python code (START)
    unsuspend_button_.publish(true);
    ros::Duration(0.5).sleep();
    unsuspend_button_.publish(false);
    //Added to be compatible with our Python code (END)
    break;
  default:
    command_valid = false;
    break;

  }
  if(command_valid)
  {
    if(command_rtn)
    {
      ss << "Successful transition request command: " << command_int;
      //ROS_INFO_STREAM(ss.str());
      res.success = true;
      res.error_code = res.SUCCESS;
      res.message = ss.str();
    }
    else
    {
      ss << "Invalid transition request command: " << command_int;
      //ROS_ERROR_STREAM(ss.str());
      res.success = false;
      res.error_code = res.INVALID_TRANSITION_REQUEST;
      res.message = ss.str();
    }
  }
  else
  {
    ss << "Unrecognized transition request command: " << command_int;
    //ROS_ERROR_STREAM(ss.str());
    res.success = false;
    res.error_code = res.UNRECGONIZED_REQUEST;
    res.message = ss.str();
  }

}

} // namespace kitsune_robot
