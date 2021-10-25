/*
* Copyright (C) Evocortex GmbH - All Rights Reserved
* 
* Unauthorized copying of this file, via any medium is strictly prohibited
* Proprietary and confidential
*
* Written by Dr. Christian Pfitzner <christian.pfitzner@evocortex.com>
* 05 2019
*/
#ifndef CHASSIS_H
#define CHASSIS_H

 
// standard includes
#include <map>
#include <array>
#include <iostream>
#include <string>
#include <cmath>

// ros specific includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

//double M_PI = 3.14159265359;
/**
 * @enum MotorID 
 */
enum class MotorID
{
  front_left, 
  front_right, 
  rear_left, 
  rear_right
}; 


/**
 * @class   Chassis
 * @author  Christian Pfitzner
 * 
 */
class Chassis
{
  public: 
    /**
     * @brief Construct a new Chassis Param object
     * 
     * @param ms2rpm 
     * @param wheelbase 
     * @param track 
     */
    Chassis(double ms2rpm = 0.0, double wheelbase = 0.0, double track = 0.0)  
    : _ms2rpm(ms2rpm), _wheelbase(wheelbase), _track(track)
    { 
      this->updateConfig(); 

      _motor_ids[MotorID::front_left]  = 2; 
      _motor_ids[MotorID::front_right] = 3; 
      _motor_ids[MotorID::rear_left]   = 0; 
      _motor_ids[MotorID::rear_right]  = 1; 

    }

    /**
     * @brief Destroy the Chassis Param object
     */
    ~Chassis(void) { }

    /**
     * @brief Get the Rpm object
     * @param twist 
     * @return std::array<int, 4> 
     */
    std::array<int, 4> getRpm(const geometry_msgs::msg::Twist::SharedPtr twist) const
    {
     // RCLCPP_DEBUG("linear x: " << twist->linear.x << " linear y: " << twist->linear.y);

      const auto  rpmFwd        =  twist->linear.x   / sin(M_PI/4.0) * _ms2rpm;
      const auto  rpmLeft       =  twist->linear.y   / sin(M_PI/4.0) * _ms2rpm;
      const auto  rpmOmega      =  twist->angular.z  / sin(M_PI/4.0) * _tangentialFactor * _leverage * _ms2rpm;
 
      const auto frontLeftID    = _motor_ids.at(MotorID::front_left); 
      const auto frontRightID   = _motor_ids.at(MotorID::front_right); 
      const auto rearLeftID     = _motor_ids.at(MotorID::rear_left); 
      const auto rearRightID    = _motor_ids.at(MotorID::rear_right); 

      std::array<int, 4> rpm    = {{0, 0, 0, 0}};
      // equations taken from http://robotsforroboticists.com/drive-kinematics/
      rpm[frontLeftID]          = rpmFwd - rpmLeft - rpmOmega; //  - _lift_compensation_rpm_active;
      rpm[frontRightID]         = rpmFwd + rpmLeft + rpmOmega; //  + _lift_compensation_rpm_active;
      rpm[rearLeftID]           = rpmFwd + rpmLeft - rpmOmega; //  + _lift_compensation_rpm_active;
      rpm[rearRightID]          = rpmFwd - rpmLeft + rpmOmega; //  - _lift_compensation_rpm_active;

      // switch direction for all drives
      rpm[frontLeftID]         *= (-1);
      rpm[frontRightID]        *= (-1);  
      rpm[rearLeftID]          *= (-1);
      rpm[rearRightID]         *= (-1);

      return rpm; 
    }




    // setters
    /**
     * @brief Set the Ms2 Rrpm object
     * @param ms2rpm 
     */
    void setMs2Rrpm(const double ms2rpm)      { _ms2rpm    = ms2rpm;    this->updateConfig();}
    /**
     * @brief Set the Wheelbase object
     * @param wheelbase 
     */
    void setWheelbase(const double wheelbase) { _wheelbase = wheelbase; this->updateConfig();}
    /**
     * @brief Set the Track object
     * @param track 
     */
    void setTrack(const double track)         { _track     = track;     this->updateConfig();}


    // getters
    /**
     * @brief Get the Ms2 Rpm object
     * @return double 
     */
    double getMs2Rpm(void) const              { return _ms2rpm; }
    /**
     * @brief Get the Wheelbase object
     * @return double 
     */
    double getWheelbase(void) const           { return _wheelbase; }
    /**
     * @brief Get the Track object
     * @return double 
     */
    double getTrack(void) const               { return _track; }
    /**
     * @brief Get the Tangential Factor object
     * @return double 
     */
    double getTangentialFactor(void) const    { return _tangentialFactor; }
    /**
     * @brief Get the Leverage object
     * @return double 
     */
    double getLeverage(void) const            { return _leverage; }
    /**
     * @brief Get the Id Front Left object
     * @return unsigned int 
     */
    unsigned int getIdFrontLeft(void) const   { return _motor_ids.at(MotorID::front_left);  }
    /**
     * @brief Get the Id Front Right object
     * @return unsigned int 
     */
    unsigned int getIdFrontRight(void) const  { return _motor_ids.at(MotorID::front_right); }
    /**
     * @brief Get the Id Rear Left object
     * @return unsigned int 
     */
    unsigned int getIdRearLeft(void) const    { return _motor_ids.at(MotorID::rear_left);   }
    /**
     * @brief Get the Id Rear Right object
     * @return unsigned int 
     */
    unsigned int getIdRearRight(void) const   { return _motor_ids.at(MotorID::rear_right);  }

    
    /**
     * @brief ostream operator overload
     * 
     * @param os 
     * @param c 
     * @return ostream& 
     */
    friend std::ostream& operator<<(std::ostream& os, const Chassis& c);

  private:  
    /**
     * @brief Function to update the configuration. 
     * This is necessary to recalculate the tangential factor and the leverage
     */
    void updateConfig(void)
    {
      _tangentialFactor = 1.0/cos(atan2(_wheelbase, _track)-(M_PI/4.0));
      _leverage         = 1.0/cos(atan2(_wheelbase, _track)-(M_PI/4.0));
    }


  private: 
    // getters and setters for the following functions
    double _ms2rpm            = 0.0;      //!< factor to switch between meters per second and rpm
    double _wheelbase         = 0.0;      //!< wheelbase of the chassis
    double _track             = 0.0;      //!< track of the chassis

    // the following parameters are calculated by the others
    double _tangentialFactor  = 0.0;      //!< factor for the tangential
    double _leverage          = 0.0;      //!< leverage of the chassis


    std::map<MotorID, int> _motor_ids;    //!< container to map the ids of the motors

}; 

std::ostream& operator<<(std::ostream& os, const Chassis& c)
{
    os << "Configuration of Chassis: "                   << std::endl <<
            " ms2rpm:      " << c.getMs2Rpm()            << std::endl << 
            " wheelbase:   " << c.getWheelbase()         << std::endl <<
            " track:       " << c.getTrack()             << std::endl << 
            " tang. fac.:  " << c.getTangentialFactor()  << std::endl <<
            " leverage:    " << c.getLeverage()          << std::endl << 
            " front left:  " << c.getIdFrontLeft()       << std::endl <<
            " front right: " << c.getIdFrontRight()      << std::endl << 
            " rear left:   " << c.getIdRearLeft()        << std::endl << 
            " rear right:  " << c.getIdRearRight()       << std::endl; 


    return os;
}


#endif // CHASSIS
