#ifndef _JETBOT_CONTROLLER_MOTORHATDIFFDRIVE_HPP_
#define _JETBOT_CONTROLLER_MOTORHATDIFFDRIVE_HPP_

#include "DiffDrive.hpp"
#include "MotorHat.hpp"

namespace jetbot_controller
{
  class MotorHatDiffDrive : public DiffDrive
  {
  public:

    struct Calibration
    {
      double meters_per_second_10;
      double meters_per_second_05;
    };

    typedef std::unique_ptr<MotorHatDiffDrive> Ptr;
    typedef std::unique_ptr<const MotorHatDiffDrive> ConstPtr;

    static Ptr create(MotorHat::Ptr &&motor_hat, const Calibration &calibration);

    virtual void setVelocity(const double left, const double right) override;
    virtual void stop() override;

  private:
    MotorHatDiffDrive(MotorHat::Ptr &&motor_hat, const Calibration &calibration);

    MotorHat::Ptr motor_hat_;
    Calibration calibration_;
  };
}

#endif