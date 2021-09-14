#include "I2c.hpp"

#include "OsException.hpp"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

#include <sstream>

#include <cstring>

using namespace jetbot_controller;

I2c::Device::Device(const std::string &bus_path, const std::uint8_t address)
  : bus_path_(bus_path)
  , address_(address)
{
}

const std::string &I2c::Device::getBusPath() const noexcept
{
  return bus_path_;
}

std::uint8_t I2c::Device::getAddress() const noexcept
{
  return address_;
}

I2c::~I2c()
{
  close(fd_);
}

I2c::Ptr I2c::open(const Device &device)
{
  const int fd = ::open(device.getBusPath().c_str(), O_RDWR);
  if (fd < 0)
  {
    std::ostringstream o;
    o << "Failed to open I2C bus " << device.getBusPath() << ": " << strerror(errno);
    throw std::runtime_error(o.str());
  }

  if (ioctl(fd, I2C_SLAVE, device.getAddress()) < 0)
  {
    std::ostringstream o;
    o << "Failed to set I2C slave address "
      << static_cast<int>(device.getAddress())
      << " on bus "
      << device.getBusPath()
      << ": "
      << strerror(errno);
    throw std::runtime_error(o.str());
  }

  return Ptr(new I2c(fd, device.getAddress()));
}

std::size_t I2c::write(const std::uint8_t *const data, const std::size_t length)
{
  ssize_t ret = ::write(fd_, data, length);
  if (ret < 0) throw OsException(errno);
  return ret;
}

std::size_t I2c::write(const std::uint8_t reg, const std::uint8_t *const data, const std::size_t length)
{
  std::uint8_t *buf = new std::uint8_t[length + 1];
  buf[0] = reg;
  std::memcpy(buf + 1, data, length);
  const std::size_t ret = write(buf, length + 1);
  delete[] buf;
  return ret;
}

std::size_t I2c::write(const std::uint8_t reg, const std::uint8_t value)
{
  return write(reg, &value, 1);
}

std::size_t I2c::read(std::uint8_t *const data, const std::size_t length)
{
  ssize_t ret = ::read(fd_, data, length);
  if (ret < 0) throw OsException(errno);
  return ret;
}

std::size_t I2c::read(const std::uint8_t reg, std::uint8_t *const data, const std::size_t length)
{
  i2c_rdwr_ioctl_data rdwr;
  rdwr.msgs = new i2c_msg[2];
  rdwr.nmsgs = 2;
  
  rdwr.msgs[0].addr = address_;
  rdwr.msgs[0].flags = 0;
  rdwr.msgs[0].len = 1;
  // Safety: We know this isn't modified by the kernel.
  rdwr.msgs[0].buf = const_cast<uint8_t *>(&reg);

  rdwr.msgs[1].addr = address_;
  rdwr.msgs[1].flags = I2C_M_RD;
  rdwr.msgs[1].len = length;
  rdwr.msgs[1].buf = data;

  const int res = ioctl(fd_, I2C_RDWR, &rdwr);

  delete[] rdwr.msgs;
  if (res < 0) throw OsException(errno);
  return res;
}

std::size_t I2c::read(const std::uint8_t reg)
{
  std::uint8_t ret = 0;
  read(reg, &ret, 1);
  return ret;
}

std::size_t I2c::mask(const std::uint8_t reg, const std::uint8_t mask)
{
  return write(reg, read(reg) & mask);
}

I2c::I2c(const int fd, const std::uint8_t address)
  : fd_(fd)
  , address_(address)
{
}