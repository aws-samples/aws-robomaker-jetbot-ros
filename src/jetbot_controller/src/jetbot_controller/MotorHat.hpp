#ifndef _JETBOT_CONTROLLER_MOTOR_HAT_HPP_
#define _JETBOT_CONTROLLER_MOTOR_HAT_HPP_

#include "I2c.hpp"

#include <memory>

namespace jetbot_controller
{
  class MotorHat
  {
  public:
    typedef std::unique_ptr<MotorHat> Ptr;
    typedef std::unique_ptr<const MotorHat> ConstPtr;

    ~MotorHat();

    static Ptr open(const I2c::Device &device);

    void setPwmFrequency(const double freq);
    void setPin(const std::uint8_t pin, const bool on);
    void setPwm(const std::uint8_t channel, const std::uint16_t on, const std::uint16_t off);
    void setAllPwm(const std::uint16_t on, const std::uint16_t off);

  private:
    MotorHat(I2c::Ptr &&i2c);

    I2c::Ptr i2c_;
  };
}

#endif