/*
 * JoyToVel.cpp
 *
 *  Created: Jul, 2020
 *      Author: Dominik Pfeiffer <dominik.pfeiffer@siemens.com>
 */

#include <numeric>
#include "JoyToVel.hpp"
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "JoyToVel.hpp"
#include <numeric>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace evo
{

  /// LifecycleTalker inheriting from rclcpp_lifecycle::LifecycleNode
  /**
 * The lifecycle talker does not like the regular "talker" node
 * inherit from node, but rather from lifecyclenode. This brings
 * in a set of callbacks which are getting invoked depending on
 * the current state of the node.
 * Every lifecycle node has a set of services attached to it
 * which make it controllable from the outside and invoke state
 * changes.
 * Available Services as for Beta1:
 * - <node_name>__get_state
 * - <node_name>__change_state
 * - <node_name>__get_available_states
 * - <node_name>__get_available_transitions
 * Additionally, a publisher for state change notifications is
 * created:
 * - <node_name>__transition_event
 */
  class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    /// LifecycleTalker constructor
    /**
   * The lifecycletalker/lifecyclenode constructor has the same
   * arguments a regular node.
   */
    explicit LifecycleTalker(const std::string &node_name, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name,
                                          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
      // init subscribers
      _joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
          "/joy_converted", 10, std::bind(&evo::LifecycleTalker::joyCallback, this, _1));

      // init publishers
      _vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/joy_vel", 10);

      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
          "/joy", 10, std::bind(&evo::LifecycleTalker::topic_callback, this, _1));

      // timer_ = create_wall_timer(
      //     100ms, std::bind(&evo::LifecycleTalker::timer_callback, this));

      joystop_ = this->create_publisher<geometry_msgs::msg::Twist>("/joy_vel", 10);

      _enable_client = this->create_client<std_srvs::srv::SetBool>("enable_client");

      _disable_client = this->create_client<std_srvs::srv::SetBool>("disable_client");
    }

    // From JoyToVel.hpp
    /**
     * Callback function for jostick
     * @param msg
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;

    /**
     * Function to publish velocities
     */
    void publishVel(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_pub;

    /**
     * Function to enable drives
     */
    void publishEnable(void);
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _enable_client;
    /**
     * Function to disable drives
     */
    void publishDisable(void);
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _disable_client;

    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;

    void joystick_stop();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joystop_;

    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;


    std::deque<double> _avg_lin_x;          //!< averaginfg deque for linear tranlation in x direction
    std::deque<double> _avg_lin_y;          //!< averaging deque for linear translation in x direction
    std::deque<double> _avg_rot;            //!< averaging deque for rotation


    // ros::ServiceClient  _enable_client;     //!< publisher to enable and disable drives
    // ros::ServiceClient  _pause_client;      //!< client to restart autonomous behaviour
    // ros::ServiceClient  _teach_client;      //!< client to restart autonomous behaviour

    // ros::ServiceClient  _button_north; 
    // ros::ServiceClient  _button_south;
    // ros::ServiceClient  _button_west;
    // ros::ServiceClient  _button_east;  
  

    JoyConfig           _joy_config;        //!< configuration for drives

    bool                _is_initialized;    //!< boolean variable if twist is enabled
    bool                _twist_disabled;    //!< boolean variable if the publication of twist is disabled


    //ros::Timer          _timer_for_deadman; //!< timer for dead man button
    bool                _dead_man_released;
    bool                _dead_man_slow_pressed; 
    bool                _published_zero_once = false;




    // 6 west
    // 7 north



    /// Callback for walltimer in order to publish the message.
    /**
   * Callback for walltimer. This function gets invoked by the timer
   * and executes the publishing.
   * For this demo, we ask the node for its current state. If the
   * lifecycle publisher is not activate, we still invoke publish, but
   * the communication is blocked so that no messages is actually transferred.
   */
    void
    publish()
    {
      static size_t count = 0;
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

      // Print the current state for demo purposes
      if (!pub_->is_activated())
      {
        RCLCPP_INFO(
            get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
      }
      else
      {
        RCLCPP_INFO(
            get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
      }

      // We independently from the current state call publish on the lifecycle
      // publisher.
      // Only if the publisher is in an active state, the message transfer is
      // enabled and the message actually published.
      pub_->publish(std::move(msg));
    }

    /// Transition callback for state configuring
    /**
   * on_configure callback is being called when the lifecycle node
   * enters the "configuring" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "unconfigured".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
    {
      // This callback is supposed to be used for initialization and
      // configuring purposes.
      // We thus initialize and configure our publishers and timers.
      // The lifecycle node API does return lifecycle components such as
      // lifecycle publishers. These entities obey the lifecycle and
      // can comply to the current state of the node.
      // As of the beta version, there is only a lifecycle publisher
      // available.
      pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);

timer_ = this->create_wall_timer(
          1s, std::bind(&evo::LifecycleTalker::timer_callback, this));

      // timer_ = this->create_wall_timer(
      //     1s, std::bind(&LifecycleTalker::publish, this));

      RCLCPP_INFO(get_logger(), "on_configure() is called.");

      // We return a success and hence invoke the transition to the next
      // step: "inactive".
      // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
      // would stay in the "unconfigured" state.
      // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
      // this callback, the state machine transitions to state "errorprocessing".
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /// Transition callback for state activating
    /**
   * on_activate callback is being called when the lifecycle node
   * enters the "activating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "active" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "active"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &)
    {
      // We explicitly activate the lifecycle publisher.
      // Starting from this point, all messages are no longer
      // ignored but sent into the network.
      pub_->on_activate();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

      // Let's sleep for 2 seconds.
      // We emulate we are doing important
      // work in the activating phase.
      std::this_thread::sleep_for(2s);

      // We return a success and hence invoke the transition to the next
      // step: "active".
      // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
      // would stay in the "inactive" state.
      // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
      // this callback, the state machine transitions to state "errorprocessing".
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /// Transition callback for state deactivating
    /**
   * on_deactivate callback is being called when the lifecycle node
   * enters the "deactivating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "active".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "active"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &)
    {
      // We explicitly deactivate the lifecycle publisher.
      // Starting from this point, all messages are no longer
      // sent into the network.
      pub_->on_deactivate();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

      // We return a success and hence invoke the transition to the next
      // step: "inactive".
      // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
      // would stay in the "active" state.
      // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
      // this callback, the state machine transitions to state "errorprocessing".
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /// Transition callback for state cleaningup
    /**
   * on_cleanup callback is being called when the lifecycle node
   * enters the "cleaningup" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "unconfigured" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
      // In our cleanup phase, we release the shared pointers to the
      // timer and publisher. These entities are no longer available
      // and our node is "clean".
      timer_.reset();
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

      // We return a success and hence invoke the transition to the next
      // step: "unconfigured".
      // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
      // would stay in the "inactive" state.
      // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
      // this callback, the state machine transitions to state "errorprocessing".
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /// Transition callback for state shutting down
    /**
   * on_shutdown callback is being called when the lifecycle node
   * enters the "shuttingdown" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "finalized" state or stays
   * in its current state.
   * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
   * TRANSITION_CALLBACK_FAILURE transitions to current state
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state)
    {
      // In our shutdown phase, we release the shared pointers to the
      // timer and publisher. These entities are no longer available
      // and our node is "clean".
      timer_.reset();
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(
          get_name(),
          "on shutdown is called from state %s.",
          state.label().c_str());

      // We return a success and hence invoke the transition to the next
      // step: "finalized".
      // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
      // would stay in the current state.
      // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
      // this callback, the state machine transitions to state "errorprocessing".
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    

  private:
    // We hold an instance of a lifecycle publisher. This lifecycle publisher
    // can be activated or deactivated regarding on which state the lifecycle node
    // is in.
    // By default, a lifecycle publisher is inactive by creation and has to be
    // activated to publish messages into the ROS world.
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

    // We hold an instance of a timer which periodically triggers the publish function.
    // As for the beta version, this is a regular timer. In a future version, a
    // lifecycle timer will be created which obeys the same lifecycle management as the
    // lifecycle publisher.
    // std::shared_ptr<rclcpp::TimerBase> timer_;
  };



  // ##############JoyToVel Methods#######################
    JoyToVel::JoyToVel() : Node("joy_to_vel")
    {

      // init subscribers
      _joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
          "/joy_converted", 10, std::bind(&JoyToVel::joyCallback, this, _1));

      // init publishers
      _vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/joy_vel", 10);

      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
          "/joy", 10, std::bind(&JoyToVel::topic_callback, this, _1));

      timer_ = create_wall_timer(
          100ms, std::bind(&JoyToVel::timer_callback, this));

      joystop_ = this->create_publisher<geometry_msgs::msg::Twist>("/joy_vel", 10);

      _enable_client = this->create_client<std_srvs::srv::SetBool>("enable_client");

      _disable_client = this->create_client<std_srvs::srv::SetBool>("disable_client");
    }

    void JoyToVel::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
      //Joy Time
      builtin_interfaces::msg::Time joy_now;
      joy_now = msg->header.stamp;
      joytime = joy_now.sec + (joy_now.nanosec * 0.000000001);
    }

    void evo::LifecycleTalker::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
      //Joy Time
      builtin_interfaces::msg::Time joy_now;
      joy_now = msg->header.stamp;
      joytime = joy_now.sec + (joy_now.nanosec * 0.000000001);
    }

    void JoyToVel::timer_callback()
    {
      //Ros System Time
      using builtin_interfaces::msg::Time;
      rclcpp::Clock ros_clock(RCL_ROS_TIME);
      Time ros_now = ros_clock.now();
      rostime = ros_now.sec + (ros_now.nanosec * 0.000000001);
      duration = (rostime - joytime);
      // printf("rostime: %lf, joytime: %lf \n", rostime, joytime);

      if (duration > 0.15)
      {
        _out_of_sync = true;
        RCLCPP_WARN(this->get_logger(), "Emergency! Connection to controller lost!");
        geometry_msgs::msg::Twist joy_disconnect = geometry_msgs::msg::Twist();
        _vel_pub->publish(joy_disconnect);
      }
      else
      {
        _out_of_sync = false;
      }
    }

    void evo::LifecycleTalker::timer_callback()
    {
      //Ros System Time
      using builtin_interfaces::msg::Time;
      rclcpp::Clock ros_clock(RCL_ROS_TIME);
      Time ros_now = ros_clock.now();
      rostime = ros_now.sec + (ros_now.nanosec * 0.000000001);
      duration = (rostime - joytime);
      // printf("rostime: %lf, joytime: %lf \n", rostime, joytime);

      if (duration > 0.15)
      {
        _out_of_sync = true;
        RCLCPP_WARN(this->get_logger(), "Emergency! Connection to controller lost!");
        geometry_msgs::msg::Twist joy_disconnect = geometry_msgs::msg::Twist();
        _vel_pub->publish(joy_disconnect);
      }
      else
      {
        _out_of_sync = false;
      }
    }

    void JoyToVel::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      static sensor_msgs::msg::Joy::SharedPtr msg_old = msg;

      this->publishVel(msg);

      if (!msg->buttons[static_cast<int>(ButtonMap::dead_man_button)])
        _dead_man_released = true;
      else
        _dead_man_released = false;

      if (!msg->buttons[static_cast<int>(ButtonMap::dead_man_slow)])
        _dead_man_slow_pressed = true;
      else
        _dead_man_slow_pressed = false;

      this->publishVel(msg);

      if (!msg->buttons[static_cast<int>(ButtonMap::dead_man_button)])
        _dead_man_released = true;
      else

        // save the current one to the old one for rising trigger
        msg_old = msg;
    }

    void evo::LifecycleTalker::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      static sensor_msgs::msg::Joy::SharedPtr msg_old = msg;

      this->publishVel(msg);

      if (!msg->buttons[static_cast<int>(ButtonMap::dead_man_button)])
        _dead_man_released = true;
      else
        _dead_man_released = false;

      if (!msg->buttons[static_cast<int>(ButtonMap::dead_man_slow)])
        _dead_man_slow_pressed = true;
      else
        _dead_man_slow_pressed = false;

      this->publishVel(msg);

      if (!msg->buttons[static_cast<int>(ButtonMap::dead_man_button)])
        _dead_man_released = true;
      else

        // save the current one to the old one for rising trigger
        msg_old = msg;
    }

    void JoyToVel::publishVel(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      static geometry_msgs::msg::Twist twist_old = geometry_msgs::msg::Twist();
      geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();

      double v_x = msg->axes[static_cast<int>(AxesMap::trans_x)];
      double v_y = msg->axes[static_cast<int>(AxesMap::trans_y)];
      double r_z = msg->axes[static_cast<int>(AxesMap::rot_z)];

      double limit_vel = _joy_config.v_max;
      if (!_dead_man_slow_pressed)
      {
        limit_vel = 0.4 * _joy_config.v_max;
      }

      limit_value<double>(v_x, -limit_vel, limit_vel);
      limit_value<double>(v_y, -limit_vel, limit_vel);

      _avg_lin_x.push_front(v_x);
      _avg_lin_y.push_front(v_y);
      _avg_rot.push_front(r_z);

      v_x = std::accumulate(_avg_lin_x.begin(), _avg_lin_x.end(), 0.0) / _avg_lin_x.size();
      v_y = std::accumulate(_avg_lin_y.begin(), _avg_lin_y.end(), 0.0) / _avg_lin_y.size();
      r_z = std::accumulate(_avg_rot.begin(), _avg_rot.end(), 0.0) / _avg_rot.size();

      _avg_lin_x.pop_back();
      _avg_lin_y.pop_back();
      _avg_rot.pop_back();

      // apply square joystick curve
      twist.linear.x = v_x * std::abs(v_x) * _joy_config.lin_scale;
      twist.linear.y = v_y * std::abs(v_y) * _joy_config.lin_scale;
      twist.angular.z = r_z * std::abs(r_z) * _joy_config.rot_scale;

      // check if dead man switch is configured and pressed
      if (_dead_man_released)
      {
        RCLCPP_INFO(this->get_logger(), "Dead man switch is released. The robot will stop immediatly");

        if (!_published_zero_once)
        {
          RCLCPP_INFO(this->get_logger(), "Publishing zero velocity");
          auto twist = geometry_msgs::msg::Twist();
          _vel_pub->publish(twist);
          //keep publishing if controller is disconnected
          if (_out_of_sync == false)
          {
            _published_zero_once = true;
          }
        }
      }
      else
      {
        //    _timer_for_deadman.setPeriod(ros::Duration(2.0), true);
        _vel_pub->publish(twist);
      }

      if (!_dead_man_released)
      {
        _published_zero_once = false;
      }

      twist_old = twist;
    }

    void evo::LifecycleTalker::publishVel(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      static geometry_msgs::msg::Twist twist_old = geometry_msgs::msg::Twist();
      geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();

      double v_x = msg->axes[static_cast<int>(AxesMap::trans_x)];
      double v_y = msg->axes[static_cast<int>(AxesMap::trans_y)];
      double r_z = msg->axes[static_cast<int>(AxesMap::rot_z)];

      double limit_vel = _joy_config.v_max;
      if (!_dead_man_slow_pressed)
      {
        limit_vel = 0.4 * _joy_config.v_max;
      }

      limit_value<double>(v_x, -limit_vel, limit_vel);
      limit_value<double>(v_y, -limit_vel, limit_vel);

      _avg_lin_x.push_front(v_x);
      _avg_lin_y.push_front(v_y);
      _avg_rot.push_front(r_z);

      v_x = std::accumulate(_avg_lin_x.begin(), _avg_lin_x.end(), 0.0) / _avg_lin_x.size();
      v_y = std::accumulate(_avg_lin_y.begin(), _avg_lin_y.end(), 0.0) / _avg_lin_y.size();
      r_z = std::accumulate(_avg_rot.begin(), _avg_rot.end(), 0.0) / _avg_rot.size();

      _avg_lin_x.pop_back();
      _avg_lin_y.pop_back();
      _avg_rot.pop_back();

      // apply square joystick curve
      twist.linear.x = v_x * std::abs(v_x) * _joy_config.lin_scale;
      twist.linear.y = v_y * std::abs(v_y) * _joy_config.lin_scale;
      twist.angular.z = r_z * std::abs(r_z) * _joy_config.rot_scale;

      // check if dead man switch is configured and pressed
      if (_dead_man_released)
      {
        RCLCPP_INFO(this->get_logger(), "Dead man switch is released. The robot will stop immediatly");

        if (!_published_zero_once)
        {
          RCLCPP_INFO(this->get_logger(), "Publishing zero velocity");
          auto twist = geometry_msgs::msg::Twist();
          _vel_pub->publish(twist);
          //keep publishing if controller is disconnected
          if (_out_of_sync == false)
          {
            _published_zero_once = true;
          }
        }
      }
      else
      {
        //    _timer_for_deadman.setPeriod(ros::Duration(2.0), true);
        _vel_pub->publish(twist);
      }

      if (!_dead_man_released)
      {
        _published_zero_once = false;
      }

      twist_old = twist;
    }

    void JoyToVel::publishEnable(void)
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = motor::enable;

      while (!_enable_client->wait_for_service(1s))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
    }

    void JoyToVel::publishDisable(void)
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = motor::disable;

      while (!_disable_client->wait_for_service(1s))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
    }

}

int main(int argc, char** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv); //"mechanum_joy_node"
  //RCLCPP_INFO(this->get_logger(),"Starting mechanum_joy_node");

  // rclcpp::spin(std::make_shared<evo::JoyToVel>());

  // rclcpp::spin(std::make_shared<evo::LifecycleTalker>("lc_talker"));
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<evo::JoyToVel> joy_node = std::make_shared<evo::JoyToVel>();
  std::shared_ptr<evo::LifecycleTalker> lc_node = std::make_shared<evo::LifecycleTalker>("lifecycle_joytovel");

  exe.add_node(lc_node->get_node_base_interface());
  // exe.add_node(joy_node->get_node_base_interface());

  exe.spin();
  rclcpp::shutdown();

  return 0;
}
/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
// int main(int argc, char *argv[])
// {
//   // force flush of the stdout buffer.
//   // this ensures a correct sync of all prints
//   // even when executed simultaneously within the launch file.
//   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

//   rclcpp::init(argc, argv);

//   rclcpp::executors::SingleThreadedExecutor exe;

//   std::shared_ptr<evo::LifecycleTalker> lc_node =
//       std::make_shared<evo::LifecycleTalker>("lc_talker");

//   exe.add_node(lc_node->get_node_base_interface());

//   exe.spin();

//   rclcpp::shutdown();

//   return 0;
// }


