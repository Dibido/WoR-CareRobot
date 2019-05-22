#ifndef POSITION_HPP
#define POSITION_HPP

#include "EnvironmentConsts.hpp"
#include <stdexcept>

namespace environment_controller
{

  struct Position
  {

      private:
    double mX_m;
    double mY_m;
    double mZ_m;

      public:
    Position(double aX_m, double aY_m, double aZ_m)
        : mX_m(aX_m), mY_m(aY_m), mZ_m(aZ_m)
    {
      x_m();
      y_m();
      z_m();
    }

    Position(const Position& aPos)
        : mX_m(aPos.mX_m), mY_m(aPos.mY_m), mZ_m(aPos.mZ_m)
    {
    }

    double& x_m()
    {
      if (mX_m > cMaxRange || mX_m < cMinRange)
      {
        throw std::range_error("Xpos is out of range");
      }
      return mX_m;
    }

    double& y_m()
    {
      if (mY_m > cMaxRange || mY_m < cMinRange)
      {
        throw std::range_error("Ypos is out of range");
      }
      return mY_m;
    }

    double& z_m()
    {
      if (mZ_m > cMaxRange || mZ_m < cMinRange)
      {
        throw std::range_error("Zpos is out of range");
      }
      return mZ_m;
    }
  };
} // namespace environment_controller

#endif // POSITION_HPP