#include "environment_controller/Rotation.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

#include <cmath>

namespace environment_controller
{

  Rotation::Rotation(double aRoll_rad,
                     double aPitch_rad,
                     double aYaw_rad,
                     double aQuaternion)
      : mRoll_rad(aRoll_rad),
        mPitch_rad(aPitch_rad),
        mYaw_rad(aYaw_rad),
        mQuaternion(aQuaternion)
  {
    roll_rad();
    pitch_rad();
    yaw_rad();
    quaternion();
  }

  double Rotation::roll_rad()
  {
    return fmod(mRoll_rad, M_PI);
  }

  const double& Rotation::roll_rad() const
  {
    return mRoll_rad;
  }

  double Rotation::pitch_rad()
  {
    return fmod(mPitch_rad, M_PI);
  }

  const double& Rotation::pitch_rad() const
  {
    return mPitch_rad;
  }

  double Rotation::yaw_rad()
  {
    return fmod(mYaw_rad, M_PI);
  }

  const double& Rotation::yaw_rad() const
  {
    return mYaw_rad;
  }

  double Rotation::quaternion()
  {
    return mQuaternion;
  }

  const double& Rotation::quaternion() const
  {
    return mQuaternion;
  }

} // namespace environment_controller
