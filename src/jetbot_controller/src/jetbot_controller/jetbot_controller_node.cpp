#include <controller_manager/controller_manager.h>

#include "I2c.hpp"

#include <array>
#include <chrono>
#include <thread>

#include "I2c.hpp"
#include "MotorHat.hpp"
#include "MotorHatDiffDrive.hpp"
#include "Jetbot.hpp"

using namespace jetbot_controller;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jetbot_controller");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string i2c_bus_path;
  pnh.param<std::string>("i2c_bus_path", i2c_bus_path, "/dev/i2c-1");

  int i2c_address;
  pnh.param<int>("i2c_address", i2c_address, 0x60);

  const double rate_hz = pnh.param("rate", 100.0);

  MotorHatDiffDrive::Calibration calibration;
  calibration.meters_per_second_10 = pnh.param("meters_per_second_10", 1.0);
  calibration.meters_per_second_05 = pnh.param("meters_per_second_05", 1.0);

  const I2c::Device motor_hat_device(i2c_bus_path, static_cast<std::uint8_t>(i2c_address));
  auto diff_drive = MotorHatDiffDrive::create(MotorHat::open(motor_hat_device), calibration);
  Jetbot robot(std::move(diff_drive));

  controller_manager::ControllerManager cm(&robot, nh);

  ros::Time last_read;
  ros::Time last_update;
  ros::Time last_write;

  ros::Rate rate(rate_hz);
  while (ros::ok())
  {
    const ros::Time read_time = ros::Time::now();
    robot.read(read_time, last_read - read_time);
    last_read = read_time;

    const ros::Time update_time = ros::Time::now();
    cm.update(update_time, last_update - update_time);
    last_update = update_time;

    const ros::Time write_time = ros::Time::now();
    robot.write(write_time, last_write - write_time);
    last_write = write_time;

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
