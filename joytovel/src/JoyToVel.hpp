/*
 * JoyToVel.hpp
 *
 *  Created: Jul, 2020
 *      Author: Dominik Pfeiffer <dominik.pfeiffer@siemens.com>
 */

#include <memory>
#include <unistd.h> 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>
#include <math.h>       /* pow */
// joytovel
#ifndef EVO_JOY_SRC_JOYTOVEL_H_
#define EVO_JOY_SRC_JOYTOVEL_H_

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/byte.hpp>

#include <deque>

double rostime=0;
double joytime=0;
double duration=0;
bool _out_of_sync=false;
builtin_interfaces::msg::Time joy_now;
builtin_interfaces::msg::Time ros_now;
// builtin_interfaces::msg::Time Duration;


using std::placeholders::_1;
using namespace std::chrono_literals;


/**
 * @namespace evo
 *
 */
namespace evo {


/**
  * @namespace motor
  */
namespace motor
{
   constexpr bool enable  = true;
   constexpr bool disable = false;
}

/**
 * @enum ButtonMap
 */
enum class ButtonMap
{
  enable          = 0,      //!< enable
  disable         = 1,      //!< disable
  lift_up         = 2,      //!< lift_up
  lift_down       = 3,      //!< lift_down
  pause           = 6,
  teach           = 4,
  disable_twist   = 10,
  dead_man_button = 5,
  dead_man_slow   = 7
};


/**
 * @enum AxesMap
 */
enum class AxesMap
{
  trans_x = 1,//!< trans_x
  trans_y = 0,//!< trans_y
  rot_z   = 2, //!< rot_z


  button_west = 6,
  button_north = 7
};


/**
 * @struct JoyConfig
 *
 */
struct JoyConfig
{
  float rot_scale                    = 1.0;
  float lin_scale                    = 1.0;
  float v_max                        = 1.0;

  unsigned int averaging_window_size = 20;

  float lift_v_max                   = 127;
  float delta_lin_x                  = 0.01; 
  float delta_lin_y                  = 0.01; 

};




/**'
 * @class   JoyToVel
 * @author  Christian Pfitzner
 */
class JoyToVel : public rclcpp::Node
{
  public:
    JoyToVel();

    /**
     * Spin function of ros node
     */
    void run(void);
  private:
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

};



/**
 * Converter Function to limit a variable
 * @param var     variable
 * @param min     minimum value
 * @param max     maximum value
 */
template <typename T>
static void limit_value(T& var, const T min, const T max)
{
  if(min >= max) {
    std::cout << "error: minimum must be smaller than maximum" << std::endl;
    return;
  }

  if(var > max) var = max;
  if(var < min) var = min;
}



} /* namespace evo */

#endif /* EVO_JOY_SRC_JOYTOVEL_H_ */
