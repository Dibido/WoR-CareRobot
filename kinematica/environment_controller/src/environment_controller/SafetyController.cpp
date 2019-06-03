#include "environment_controller/SafetyController.hpp"
#include "environment_controller/EnvironmentConsts.hpp"
#include "environment_controller/EnvironmentController.hpp"

#include <cmath>
namespace environment_controller
{
  SafetyController::SafetyController(
      const std::shared_ptr<EnvironmentController>& aEnvironmentController)
      : mEnvironmentController(aEnvironmentController)
  {
  }

  void SafetyController::areObstaclesAThreat(const Obstacles& aObstacles)
  {
    bool lAllObstaclesSafe = true;
    for (const Obstacle& lObstacle : aObstacles)
    {
      if (isObstacleToClose(lObstacle))
      {
        lAllObstaclesSafe = false;
        mEnvironmentController->executeHardstop(true);
      }
    }
    if (lAllObstaclesSafe)
    {
      mEnvironmentController->executeHardstop(false);
    }
    mEnvironmentController->provideObstacles(aObstacles);
  }

  bool SafetyController::isObstacleToClose(const Obstacle& aObstacle)
  {
    bool lToClose = false;
    double lDeltaX = abs(cRobotX_m - aObstacle.position().x_m());
    double lDeltaY = abs(cRobotY_m - aObstacle.position().y_m());
    double minRadiusCircle =
        sqrt(pow(aObstacle.depth_m(), 2) + pow(aObstacle.width_m(), 2)) / 2;

    // 2D distance between the middlepoint of the obstacle and robotarm
    double lDistance = sqrt(pow(lDeltaX, 2) + pow(lDeltaY, 2));

    // Check if the distance between the middlepoints is larger than the two
    // radii combined.
    if (lDistance < cMinDistanceToRobotarm_m + minRadiusCircle)
    {
      lToClose = true;
    }
    return lToClose;
  };

} // namespace environment_controller
