#include "environment_controller/Sensor.hpp"

namespace environment_controller
{

  Sensor::Sensor(const uint8_t aSensorID, const Pose& aPose)
      : mSensorID(aSensorID), mPose(aPose)
  {
    pose();
    sensorID();
  }

  Pose& Sensor::pose()
  {
    mPose.position();
    mPose.rotation();
    return mPose;
  }

  const Pose& Sensor::pose() const
  {
    mPose.position();
    mPose.rotation();
    return mPose;
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
