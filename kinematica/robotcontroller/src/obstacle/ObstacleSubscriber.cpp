#include "kinematica_msgs/Object.h"
#include "obstacle/Obstacle.hpp"
#include "obstacle/ObstacleSubsciber.hpp"
#include "robotcontroller_msgs/Stop.h"

namespace obstacle
{
  ObstacleSubsciber::ObstacleSubsciber(std::string& aSubName)
      : mPublis(mHandlePub,"robot_stop",1000)
  {
    mSubscriber = mHandle.subscribe(
        aSubName, QUEUE_SIZE, &obstacle::ObstacleSubsciber::obstaclesCallback,
        this);
  }

  void ObstacleSubsciber::obstaclesCallback(
      const kinematica_msgs::ObstaclesConstPtr& aMsg)
  {
    mPublis.publish(true);
  }
} // namespace obstacle
