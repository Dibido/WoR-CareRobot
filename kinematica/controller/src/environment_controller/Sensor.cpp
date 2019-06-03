#include "environment_controller/Sensor.hpp"

namespace environment_controller
{

  Sensor::Sensor(const Position& aPosition,
                 const Rotation& aRotation,
                 const uint8_t aSensorID)
      : mPosition(aPosition), mRotation(aRotation), mSensorID(aSensorID)
  {
    roll_rad();
    pitch_rad();
    yaw_rad();
    quaternion();
  }

  Position& position()
  {
    mPosition.x_m();
    mPosition.y_m();
    mPosition.z_m();
    return mPosition;
  }

  const Position& position() const
  {
    mPosition.x_m();
    mPosition.y_m();
    mPosition.z_m();
    return mPosition;
  }

  Rotation& rotation()
  {
    mRotation.roll_rad();
    mRotation.pitch_rad();
    mRotation.yaw_rad();
    mRotation / quaternion();
    return mRotation;
  }

  const Rotation& rotation() const
  {
    mRotation.roll_rad();
    mRotation.pitch_rad();
    mRotation.yaw_rad();
    mRotation / quaternion();
    return mRotation;
  }

  uint8_t& sensorID()
  {
    return mSensorID;
  }

  const uint8_t& sensorID() const
  {
    return mSensorID;
  }

} // namespace environment_controller
