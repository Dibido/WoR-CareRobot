#include "kinematica_msgs/Object.h"
#include "obstacle/Obstacle.hpp"
#include "obstacle/ObstacleSubsciber.hpp"

namespace obstacle
{
  ObstacleSubsciber::ObstacleSubsciber(const std::shared_ptr<Safety> aSafety,
                                       std::string& aSubName)
      : mSafety(aSafety)
  {
    mSubscriber = mHandle.subscribe(
        aSubName, QUEUE_SIZE, &obstacle::ObstacleSubsciber::obstaclesCallback,
        this);
  }

  void ObstacleSubsciber::obstaclesCallback(
      const kinematica_msgs::ObstaclesConstPtr& aMsg)
  {
    for (size_t i = 0; i < aMsg.size(); ++i)
    {
    }
  }
} // namespace obstacle
