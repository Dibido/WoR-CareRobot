#include "environment_controller/Sensor.hpp"

namespace environment_controller
{

  Sensor::Sensor(const Position& aPosition,
                 const Rotation& aRotation,
                 const uint8_t aSensorID)
      : mPosition(aPosition), mRotation(aRotation), mSensorID(aSensorID)
  {
    position();
    rotation();
    sensorID();
  }

  Position& Sensor::position()
  {
    mPosition.x_m();
    mPosition.y_m();
    mPosition.z_m();
    return mPosition;
  }

  const Position& Sensor::position() const
  {
    mPosition.x_m();
    mPosition.y_m();
    mPosition.z_m();
    return mPosition;
  }

  Rotation& Sensor::rotation()
  {
    mRotation.roll_rad();
    mRotation.pitch_rad();
    mRotation.yaw_rad();
    mRotation.quaternion();
    return mRotation;
  }

  const Rotation& Sensor::rotation() const
  {
    mRotation.roll_rad();
    mRotation.pitch_rad();
    mRotation.yaw_rad();
    mRotation.quaternion();
    return mRotation;
  }

  uint8_t Sensor::sensorID()
  {
    return mSensorID;
  }

  const uint8_t& Sensor::sensorID() const
  {
    return mSensorID;
  }

} // namespace environment_controller
