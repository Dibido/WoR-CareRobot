#include "environment_controller/Cup.hpp"

namespace environment_controller
{

  Cup::Cup(const Object& aObject, const ros::Time& aTimeOfArrival)
      : mObject(aObject), mTimeOfArrival(aTimeOfArrival)
  {
    object();
    timeOfArrival();
  }

  Object& Cup::object()
  {
    mObject.position();
    mObject.height_m();
    mObject.width_m();
    mObject.depth_m();
    mObject.direction_rad();
    mObject.speed_ms();
    mObject.measurementTime();
    mObject.sensorId();
    return mObject;
  }
  const Object& Cup::object() const
  {
    mObject.position();
    mObject.height_m();
    mObject.width_m();
    mObject.depth_m();
    mObject.direction_rad();
    mObject.speed_ms();
    mObject.measurementTime();
    mObject.sensorId();
    return mObject;
  }

  ros::Time& Cup::timeOfArrival()
  {
    // if (mTimeOfArrival < ros::Time::now())
    // {
    //   throw std::range_error("time of arrival is in the past");
    // }
    return mTimeOfArrival;
  }

  const ros::Time& Cup::timeOfArrival() const
  {
    // if (mTimeOfArrival < ros::Time::now())
    // {
    //   throw std::range_error("time of arrival is in the past");
    // }
    return mTimeOfArrival;
  }

} // namespace environment_controller