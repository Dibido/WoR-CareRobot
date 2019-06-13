#include "environment_controller/Object.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

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
        mDirecton_rad(aDirection),
        mSpeed_ms(aSpeed),
        mMeasurementTime(aMeasurementTime),
        mSensorId(aSensorId)
  {
    position();
    height_m();
    width_m();
    depth_m();
    direction_rad();
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

  const Position& Object::position() const
  {
    mPosition.x_m();
    mPosition.y_m();
    mPosition.z_m();
    return mPosition;
  }

  double& Object::height_m()
  {
    if (mHeight_m >= cMaxRange_m || mHeight_m <= cMinRange_m)
    {
      throw std::range_error("height is out of range");
    }
    return mHeight_m;
  }

  const double& Object::height_m() const
  {
    if (mHeight_m >= cMaxRange_m || mHeight_m <= cMinRange_m)
    {
      throw std::range_error("height is out of range");
    }
    return mHeight_m;
  }

  double& Object::width_m()
  {
    if (mWidth_m >= cMaxRange_m || mWidth_m <= cMinRange_m)
    {
      throw std::range_error("width is out of range");
    }
    return mWidth_m;
  }

  const double& Object::width_m() const
  {
    if (mWidth_m >= cMaxRange_m || mWidth_m <= cMinRange_m)
    {
      throw std::range_error("width is out of range");
    }
    return mWidth_m;
  }

  double& Object::depth_m()
  {
    if (mDepth_m >= cMaxRange_m || mDepth_m <= cMinRange_m)
    {
      throw std::range_error("depth is out of range");
    }
    return mDepth_m;
  }

  const double& Object::depth_m() const
  {
    if (mDepth_m >= cMaxRange_m || mDepth_m <= cMinRange_m)
    {
      throw std::range_error("depth is out of range");
    }
    return mDepth_m;
  }

  double& Object::direction_rad()
  {
    if (mDirecton_rad <= cLow_rad || mDirecton_rad > cHigh_rad)
    {
      throw std::range_error("direction is out of range");
    }
    return mDirecton_rad;
  }

  const double& Object::direction_rad() const
  {
    if (mDirecton_rad <= cLow_rad || mDirecton_rad > cHigh_rad)
    {
      throw std::range_error("direction is out of range");
    }
    return mDirecton_rad;
  }

  double& Object::speed_ms()
  {
    if (mSpeed_ms >= cTooFast_ms || mSpeed_ms < cTooSlow_ms)
    {
      throw std::range_error("speed is out of range");
    }
    return mSpeed_ms;
  }

  const double& Object::speed_ms() const
  {
    if (mSpeed_ms >= cTooFast_ms || mSpeed_ms < cTooSlow_ms)
    {
      throw std::range_error("speed is out of range");
    }
    return mSpeed_ms;
  }

  ros::Time& Object::measurementTime()
  {
    return mMeasurementTime;
  }

  const ros::Time& Object::measurementTime() const
  {
    return mMeasurementTime;
  }

  uint8_t& Object::sensorId()
  {
    return mSensorId;
  }

  const uint8_t& Object::sensorId() const
  {
    return mSensorId;
  }

} // namespace environment_controller
