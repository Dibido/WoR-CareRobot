#include "controller/Context.hpp"
#include "environment_controller/CupSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"
#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/ObstaclesSubscriber.hpp"
#include "environment_controller/SafetyController.hpp"
#include "ros/ros.h"
#include <memory>
#include <stdlib.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "main_application");
  std::shared_ptr<controller::Context> lContext =
      std::make_shared<controller::Context>();

  std::shared_ptr<environment_controller::EnvironmentController>
      lEnvironmentController =
          std::make_shared<environment_controller::EnvironmentController>(
              lContext);

  environment_controller::SafetyController lSafetyController =
      environment_controller::SafetyController(lEnvironmentController);

  environment_controller::ObstaclesSubscriber lObstacleSubscriber =
      environment_controller::ObstaclesSubscriber(
          std::make_shared<environment_controller::SafetyController>(
              lSafetyController),
          environment_controller::cObstacleTopicName);

  environment_controller::CupSubscriber lCupSubscriber =
      environment_controller::CupSubscriber(
          environment_controller::cCupTopicName, lEnvironmentController);
  ros::spin();

  return 0;
}
