#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

namespace obstacle
{
  struct Obstacle
  {
    double mX;
    double mY;
    double mZ;
    double mWidth;
    double mHeight;

    bool operator==(const Obstacle& rhs) const
    {
      return mX == rhs.mX && mY == rhs.mY && mZ == rhs.mZ &&
             mWidth == rhs.mWidth && mHeight == rhs.mHeight;
    }
  };
} // namespace obstacle

#endif // OBSTACLE_HPP