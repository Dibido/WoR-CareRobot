#include "environment_controller/Object.hpp"

namespace environment_controller
{
  Object::Object(const Position& aPosition,
                 double aHeight,
                 double aWidth,
                 double aDepth,
                 double aDirection,
                 double aSpeed,
                 const ros::Time& aMeasurementTime,
                 uint8_t aSensorId)
      : mPosition(aPosition),
        mHeight_m(aHeight),
        mWidth_m(aWidth),
        mDepth_m(aDepth),
        mSpeed_ms(aSpeed),
        mMeasurementTime(aMeasurementTime),
        mSensorId(aSensorId)
  {
    position();
    height_m();
    width_m();
    depth_m();
    speed_ms();
    measurementTime();
  }

  Position& Object::position()
  {
    mPosition.x_m();
    mPosition.y_m();
    mPosition.z_m();
    return mPosition;
  }

  double& Object::height_m()
  {
    if (mHeight_m >= cMaxRange || mHeight_m <= cMinRange)
    {
      throw std::range_error("height is out of range");
    }
    return mHeight_m;
  }

  double& Object::width_m()
  {
    if (mWidth_m >= cMaxRange || mWidth_m <= cMinRange)
    {
      throw std::range_error("width is out of range");
    }
    return mWidth_m;
  }

  double& Object::depth_m()
  {
    if (mDepth_m >= cMaxRange || mDepth_m <= cMinRange)
    {
      throw std::range_error("depth is out of range");
    }
    return mDepth_m;
  }

  double& Object::speed_ms()
  {
    if (mSpeed_ms >= cToFast_ms || mSpeed_ms <= cToSlow_ms)
    {
      throw std::range_error("speed is out of range");
    }
    return mSpeed_ms;
  }

  ros::Time& Object::measurementTime()
  {
    if (mMeasurementTime == ros::Time(0))
    {
      throw std::range_error("time is out of range");
    }
    return mMeasurementTime;
  }
  uint8_t& Object::sensorId()
  {
    return mSensorId;
  }

} // namespace environment_controller