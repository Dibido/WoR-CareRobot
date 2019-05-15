#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

namespace obstacle
{
  struct Obstacle
  {
    double x;
    double y;
    double z;
    double width;
    double height;

    bool operator==(const Obstacle& rhs) const
    {
      return x == rhs.x && y == rhs.y && z == rhs.z && width == rhs.width &&
             height == rhs.height;
    }
  };
} // namespace obstacle

#endif // OBSTACLE_HPP