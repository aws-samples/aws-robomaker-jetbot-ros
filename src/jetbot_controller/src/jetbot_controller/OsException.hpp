#ifndef _JETBOT_CONTROLLER_OSEXCEPTION_HPP_
#define _JETBOT_CONTROLLER_OSEXCEPTION_HPP_

#include <stdexcept>

namespace jetbot_controller
{
  class OsException : public std::exception
  {
  public:
    // Note: `errno` is a preprocessor definition
    OsException(const int errno_);
    
    int getErrno() const noexcept;

    const char *what() const noexcept override;

  private:
    int errno_;
  };
}

#endif