#include "environment_controller/Rotation.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

#include <cmath>

namespace environment_controller
{

  Rotation::Rotation(double aX, double aY, double aZ, double aW)
      : mX(aX), mY(aY), mZ(aZ), mW(aW)
  {
    x();
    y();
    z();
    w();
  }

  double Rotation::x()
  {
    if (mX > 1 || mX < -1)
    {
      throw std::range_error("x is out of range");
    }
    return mX;
  }

  const double& Rotation::x() const
  {
    if (mX > 1 || mX < -1)
    {
      throw std::range_error("x is out of range");
    }
    return mX;
  }

  double Rotation::y()
  {
    if (mY > 1 || mY < -1)
    {
      throw std::range_error("y is out of range");
    }
    return mY;
  }

  const double& Rotation::y() const
  {
    if (mY > 1 || mY < -1)
    {
      throw std::range_error("y is out of range");
    }
    return mY;
  }

  double Rotation::z()
  {
    if (mZ > 1 || mZ < -1)
    {
      throw std::range_error("z is out of range");
    }
    return mZ;
  }

  const double& Rotation::z() const
  {
    if (mZ > 1 || mZ < -1)
    {
      throw std::range_error("z is out of range");
    }
    return mZ;
  }

  double Rotation::w()
  {
    if (mW > 1 || mW < -1)
    {
      throw std::range_error("w is out of range");
    }
    return mW;
  }

  const double& Rotation::w() const
  {
    if (mW > 1 || mW < -1)
    {
      throw std::range_error("w is out of range");
    }
    return mW;
  }

} // namespace environment_controller
