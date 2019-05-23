#include "environment_controller/ObstaclesSubscriber.hpp"
#include "environment_controller/SafetyController.hpp"
#include "kinematica_msgs/Object.h"
#include "environment_controller/EnvironmentConsts.hpp"

namespace environment_controller
{
  ObstaclesSubscriber::ObstaclesSubscriber(
      const std::shared_ptr<SafetyController>& aSafetyController,
      const std::string& aSubName)
      : mSafetyController(aSafetyController),
        mSubscriber(mHandle.subscribe(
            aSubName,
            cQueue_size,
            &environment_controller::ObstaclesSubscriber::obstaclesCallback,
            this))
  {
  }

  void ObstaclesSubscriber::obstaclesCallback(
      const kinematica_msgs::ObstaclesConstPtr& aMsg)
  {
    Obstacles lObstacles;
    for (std::size_t i = 0; i < aMsg->obstacles.size(); ++i)
    {
      try
      {
        environment_controller::Position lPos(aMsg->obstacles[i].position_m.x,
                                              aMsg->obstacles[i].position_m.y,
                                              aMsg->obstacles[i].position_m.z);
        environment_controller::Object lObj(
            lPos, aMsg->obstacles[i].height_m, aMsg->obstacles[i].width_m,
            aMsg->obstacles[i].depth_m, aMsg->obstacles[i].direction_rad,
            aMsg->obstacles[i].speed_ms, aMsg->obstacles[i].measurementTime,
            aMsg->obstacles[i].sensorID);
        lObstacles.push_back(lObj);
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("%s", e.what());
      }
    }
    if (lObstacles.size())
      parseObstacles(lObstacles);
  }

  void ObstaclesSubscriber::parseObstacles(const Obstacles& aObstacles)
  {
    mSafetyController->areObstaclesAThreat(aObstacles);
  }
} // namespace environment_controller
