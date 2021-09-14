#include "Jetbot.hpp"

using namespace jetbot_controller;

Jetbot::Jetbot(DiffDrive::Ptr &&diff_drive)
  : diff_drive_(std::move(diff_drive))
{
  hardware_interface::JointStateHandle left_wheel_state("left_wheel_hinge", &left_.position, &left_.velocity, &left_.effort);
  state_interface_.registerHandle(left_wheel_state);

  hardware_interface::JointHandle left_wheel_handle(left_wheel_state, &left_.command);
  velocity_interface_.registerHandle(left_wheel_handle);

  hardware_interface::JointStateHandle right_wheel_state("right_wheel_hinge", &right_.position, &right_.velocity, &right_.effort);
  state_interface_.registerHandle(right_wheel_state);

  hardware_interface::JointHandle right_wheel_handle(right_wheel_state, &right_.command);
  velocity_interface_.registerHandle(right_wheel_handle);

  registerInterface(&state_interface_);
  registerInterface(&velocity_interface_);
}

Jetbot::~Jetbot()
{
  diff_drive_->stop();
}

Jetbot::Wheel::Wheel()
  : command(0.0)
  , position(0.0)
  , velocity(0.0)
  , effort(0.0)
{
}

void Jetbot::read(const ros::Time &time, const ros::Duration &period)
{
  if (time.isZero()) return;

  left_.velocity = left_.command;
  right_.velocity = right_.command;


  left_.position += period.toSec() * left_.velocity;
  right_.position += period.toSec() * right_.velocity;

  left_.effort = 0.0;
  right_.effort = 0.0;
}

void Jetbot::write(const ros::Time &time, const ros::Duration &period)
{
  diff_drive_->setVelocity(-left_.command, -right_.command);
}