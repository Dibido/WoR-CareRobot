#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

namespace obstacle
{
  struct Obstacle
  {
    double mX_m;
    double mY_m;
    double mZ_m;
    double mWidth_m;
    double mHeight_m;

    bool operator==(const Obstacle& rhs) const
    {
      return mX_m == rhs.mX_m && mY_m == rhs.mY_m && mZ_m == rhs.mZ_m &&
             mWidth_m == rhs.mWidth_m && mHeight_m == rhs.mHeight_m;
    }
  };
} // namespace obstacle

#endif // OBSTACLE_HPP