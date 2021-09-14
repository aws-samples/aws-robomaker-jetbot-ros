#include "OsException.hpp"

#include <cstring>

using namespace jetbot_controller;

OsException::OsException(const int errno_)
  : errno_(errno_)
{
}

int OsException::getErrno() const noexcept
{
  return errno_;
}

const char *OsException::what() const noexcept
{
  return strerror(errno_);
}