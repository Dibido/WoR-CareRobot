#include "environment_controller/Pose.hpp"

namespace environment_controller
{

  Pose::Pose(const Position& aPosition, const Rotation& aRotation)
      : mPosition(aPosition), mRotation(aRotation)
  {
    position();
    rotation();
  }

  Position& Pose::position()
  {
    mPosition.x_m();
    mPosition.y_m();
    mPosition.z_m();
    return mPosition;
  }

  const Position& Pose::position() const
  {
    mPosition.x_m();
    mPosition.y_m();
    mPosition.z_m();
    return mPosition;
  }

  Rotation& Pose::rotation()
  {
    mRotation.x();
    mRotation.y();
    mRotation.z();
    mRotation.w();
    return mRotation;
  }

  const Rotation& Pose::rotation() const
  {
    mRotation.x();
    mRotation.y();
    mRotation.z();
    mRotation.w();
    return mRotation;
  }
} // namespace environment_controller
