#include "environment_controller/Obstacle.hpp"
#include "environment_controller/ObstacleSubsciber.hpp"
#include "kinematica_msgs/Object.h"
#include "robotcontroller_msgs/Stop.h"

namespace environment_controller
{
  ObstacleSubsciber::ObstacleSubsciber(std::string& aSubName)
      : mSubscriber(mHandle.subscribe(
            aSubName,
            cQueueSize,
            &environment_controller::ObstacleSubsciber::obstaclesCallback,
            this))
  {
  }

  void ObstacleSubsciber::obstaclesCallback(
      const kinematica_msgs::ObstaclesConstPtr& aMsg)
  {
  }
} // namespace environment_controller
