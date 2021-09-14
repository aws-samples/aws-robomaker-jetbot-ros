#include "MotorHatDiffDrive.hpp"

#include <iostream>
#include <cmath>

namespace
{
  template<typename T>
  T clamp(T value, T min, T max)
  {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }

  const std::uint8_t LEFT_PIN_1 = 10;
  const std::uint8_t LEFT_PIN_2 = 9;

  const std::uint8_t RIGHT_PIN_1 = 11;
  const std::uint8_t RIGHT_PIN_2 = 12;

  const std::uint8_t LEFT_PWM = 8;

  const std::uint8_t RIGHT_PWM = 13;

  constexpr double WHEEL_RADIUS = 0.035;

  constexpr double TAU = 2.0 * M_PI;
  
  constexpr double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * TAU;
}

using namespace jetbot_controller;

MotorHatDiffDrive::Ptr MotorHatDiffDrive::create(MotorHat::Ptr &&motor_hat, const Calibration &calibration)
{
  return Ptr(new MotorHatDiffDrive(std::move(motor_hat), calibration));
}

void MotorHatDiffDrive::setVelocity(const double left, const double right)
{
  const double meters_per_second = calibration_.meters_per_second_10;

  const double left_speed = left * WHEEL_RADIUS / meters_per_second;
  const double right_speed = right * WHEEL_RADIUS / meters_per_second;

  const std::uint16_t left_magnitude = clamp(std::abs(left_speed), 0.0, 1.0) * 115.0;
  const std::uint16_t right_magnitude = clamp(std::abs(right_speed), 0.0, 1.0) * 115.0;

  if (left == 0.0)
  {
    motor_hat_->setPin(LEFT_PIN_1, false);
    motor_hat_->setPin(LEFT_PIN_2, false);

    motor_hat_->setPwm(LEFT_PWM, 0, 0);
  }
  else if (left > 0.0)
  {
    motor_hat_->setPin(LEFT_PIN_1, true);
    motor_hat_->setPin(LEFT_PIN_2, false);

    motor_hat_->setPwm(LEFT_PWM, left_magnitude * 16, 0);
  }
  else
  {
    motor_hat_->setPin(LEFT_PIN_1, false);
    motor_hat_->setPin(LEFT_PIN_2, true);

    motor_hat_->setPwm(LEFT_PWM, 0, left_magnitude * 16);
  }

  if (right == 0.0)
  {
    motor_hat_->setPin(RIGHT_PIN_1, false);
    motor_hat_->setPin(RIGHT_PIN_2, false);

    motor_hat_->setPwm(RIGHT_PWM, 0, 0);
  }
  else if (right > 0.0)
  {
    motor_hat_->setPin(RIGHT_PIN_1, true);
    motor_hat_->setPin(RIGHT_PIN_2, false);

    motor_hat_->setPwm(RIGHT_PWM, right_magnitude * 16, 0);
  }
  else
  {
    motor_hat_->setPin(RIGHT_PIN_1, false);
    motor_hat_->setPin(RIGHT_PIN_2, true);

    motor_hat_->setPwm(RIGHT_PWM, 0, right_magnitude * 16);
  }
}

void MotorHatDiffDrive::stop()
{
  setVelocity(0.0, 0.0);
}

MotorHatDiffDrive::MotorHatDiffDrive(MotorHat::Ptr && motor_hat, const Calibration &calibration)
  : motor_hat_(std::move(motor_hat))
  , calibration_(calibration)
{
}
