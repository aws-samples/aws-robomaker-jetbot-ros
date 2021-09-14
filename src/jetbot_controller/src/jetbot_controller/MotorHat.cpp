#include "MotorHat.hpp"

#include <thread>

namespace
{
  const std::uint8_t MODE1 = 0x00;
  const std::uint8_t MODE2 = 0x01;
  const std::uint8_t SUBADR1 = 0x02;
  const std::uint8_t SUBADR2 = 0x03;
  const std::uint8_t SUBADR3 = 0x04;
  const std::uint8_t PRESCALE = 0xFE;
  const std::uint8_t LED0_ON_L = 0x06;
  const std::uint8_t LED0_ON_H = 0x07;
  const std::uint8_t LED0_OFF_L = 0x08;
  const std::uint8_t LED0_OFF_H = 0x09;
  const std::uint8_t ALL_LED_ON_L = 0xFA;
  const std::uint8_t ALL_LED_ON_H = 0xFB;
  const std::uint8_t ALL_LED_OFF_L = 0xFC;
  const std::uint8_t ALL_LED_OFF_H = 0xFD;

  const std::uint8_t RESTART_VALUE = 0x80;
  const std::uint8_t SLEEP_VALUE = 0x10;
  const std::uint8_t ALLCALL_VALUE = 0x01;
  const std::uint8_t INVRT_VALUE = 0x10;
  const std::uint8_t OUTDRV_VALUE = 0x04;

  const std::chrono::milliseconds CLOCK_DELAY(5);
  
}

using namespace jetbot_controller;

MotorHat::~MotorHat()
{
  // Send a software reset to the board
  const std::uint8_t raw[1] = { 0x06 };
  i2c_->write(raw, 1);
}

MotorHat::Ptr MotorHat::open(const I2c::Device &device)
{
  return Ptr(new MotorHat(I2c::open(device)));
}

void MotorHat::setPwmFrequency(const double freq)
{
  double prescaleval = 25000000.0;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  std::uint8_t prescale = std::uint8_t(prescaleval + 0.5);
  std::uint8_t old_mode = i2c_->read(MODE1);
  std::uint8_t new_mode = (old_mode & 0x7F) | 0x10;
  i2c_->write(MODE1, new_mode);
  i2c_->write(PRESCALE, prescale);
  i2c_->write(MODE1, old_mode);
  std::this_thread::sleep_for(CLOCK_DELAY);
  i2c_->write(MODE1, old_mode | 0x80);
  std::this_thread::sleep_for(CLOCK_DELAY);
  old_mode = i2c_->read(MODE1);
}

void MotorHat::setPin(const std::uint8_t pin, const bool on)
{
  if (pin > 15) throw std::invalid_argument("Pin must be between 0 and 15");

  setPwm(pin, on ? 0x1000 : 0, on ? 0 : 0x1000);
}

void MotorHat::setPwm(const std::uint8_t channel, const std::uint16_t on, const std::uint16_t off)
{
  if (channel > 15) throw std::invalid_argument("Channel must be between 0 and 15");

  i2c_->write(LED0_ON_L + 4 * channel, (on >> 0) & 0xFF);
  i2c_->write(LED0_ON_H + 4 * channel, (on >> 8) & 0xFF);
  i2c_->write(LED0_OFF_L + 4 * channel, (off >> 0) & 0xFF);
  i2c_->write(LED0_OFF_H + 4 * channel, (off >> 8) & 0xFF);
}

void MotorHat::setAllPwm(const std::uint16_t on, const std::uint16_t off)
{
  i2c_->write(ALL_LED_ON_L, (on >> 0) & 0xFF);
  i2c_->write(ALL_LED_ON_H, (on >> 8) & 0xFF);
  i2c_->write(ALL_LED_OFF_L, (off >> 0) & 0xFF);
  i2c_->write(ALL_LED_OFF_H, (off >> 8) & 0xFF);
}

MotorHat::MotorHat(I2c::Ptr &&i2c)
  : i2c_(std::move(i2c))
{
  setAllPwm(0, 0);

  i2c_->write(MODE2, OUTDRV_VALUE);
  i2c_->write(MODE1, ALLCALL_VALUE);

  // Wait for clock
  std::this_thread::sleep_for(CLOCK_DELAY);

  i2c_->mask(MODE1, ~SLEEP_VALUE);

  // Wait for clock
  std::this_thread::sleep_for(CLOCK_DELAY);

  setPwmFrequency(1600);
}