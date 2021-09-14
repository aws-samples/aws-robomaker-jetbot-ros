#ifndef _JETBOT_DIFFDRIVE_HPP_
#define _JETBOT_DIFFDRIVE_HPP_

#include <memory>

namespace jetbot_controller
{
  class DiffDrive
  {
  public:
    typedef std::unique_ptr<DiffDrive> Ptr;
    typedef std::unique_ptr<const DiffDrive> ConstPtr;

    virtual ~DiffDrive();

    /**
     * @brief Set the velocity of the left and right wheels (in m/s)
     */
    virtual void setVelocity(const double left, const double right) = 0;

    /**
     * @brief Stop all wheels
     */
    virtual void stop() = 0;
  };
}

#endif