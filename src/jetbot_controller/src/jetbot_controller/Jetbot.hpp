#ifndef _JETBOT_CONTROLLER_JETBOT_HPP_
#define _JETBOT_CONTROLLER_JETBOT_HPP_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "DiffDrive.hpp"

namespace jetbot_controller
{
  class Jetbot : public hardware_interface::RobotHW
  {
  public:
    Jetbot(DiffDrive::Ptr &&diff_drive);
    ~Jetbot();

    virtual void read(const ros::Time &time, const ros::Duration &period);
    virtual void write(const ros::Time &time, const ros::Duration &period);

  private:
    DiffDrive::Ptr diff_drive_;

    struct Wheel
    {
      Wheel();

      double command;
      double position;
      double velocity;
      double effort;
    };

    Wheel left_;
    Wheel right_;

    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::VelocityJointInterface velocity_interface_;
  };
}

#endif