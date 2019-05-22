#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "Position.hpp"
#include "ros/time.h"

namespace environment_controller
{

  struct Object
  {
      private:
    Position mPosition;
    double mHeight_m;
    double mWidth_m;
    double mDepth_m;
    double mDirecton_rad;
    double mSpeed_ms;
    ros::Time mMeasurementTime;
    uint8_t mSensorId;

      public:
    Object(const Position& aPosition,
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

    Position& position()
    {
      mPosition.x_m();
      mPosition.y_m();
      mPosition.z_m();
      return mPosition;
    }

    double& height_m()
    {
      if (mHeight_m > cMaxRange || mHeight_m < cMinRange)
      {
        throw std::range_error("height is out of range");
      }
      return mHeight_m;
    }

    double& width_m()
    {
      if (mWidth_m > cMaxRange || mWidth_m < cMinRange)
      {
        throw std::range_error("width is out of range");
      }
      return mWidth_m;
    }

    double& depth_m()
    {
      if (mDepth_m > cMaxRange || mDepth_m < cMinRange)
      {
        throw std::range_error("depth is out of range");
      }
      return mDepth_m;
    }

    double& speed_ms()
    {
      if (mSpeed_ms > cToFast_ms || mSpeed_ms < cToSlow_ms)
      {
        throw std::range_error("speed is out of range");
      }
      return mSpeed_ms;
    }

    ros::Time& measurementTime()
    {
      if (mMeasurementTime == ros::Time(0))
      {
        throw std::range_error("time is out of range");
      }
      return mMeasurementTime;
    }

    uint8_t& sensorId()
    {
      return mSensorId;
    }
  };

} // namespace environment_controller

#endif // OBJECT_HPP