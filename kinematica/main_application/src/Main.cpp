#include "controller/Context.hpp"
#include "environment_controller/CupSubscriber.hpp"
#include "environment_controller/DropTableSubscriber.hpp"
#include "environment_controller/EmergencyStopSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"
#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/GoalSubscriber.hpp"
#include "environment_controller/ObstaclesSubscriber.hpp"
#include "environment_controller/ReleaseTimeSubscriber.hpp"
#include "environment_controller/SafetyController.hpp"
#include "environment_controller/SensorSubscriber.hpp"
#include "franka_controller/FrankaFeedback.hpp"
#include "ros/ros.h"
#include <memory>
#include <stdlib.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "main_application");
  ros::start();

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
          lEnvironmentController, environment_controller::cObstacleTopicName);

  environment_controller::CupSubscriber lCupSubscriber =
      environment_controller::CupSubscriber(
          environment_controller::cCupTopicName, lEnvironmentController);

  environment_controller::GoalSubscriber lGoalSubscriber =
      environment_controller::GoalSubscriber(
          environment_controller::cGoalPositionTopicName,
          lEnvironmentController);

  environment_controller::DropTableSubscriber lDropTableSubscriber =
      environment_controller::DropTableSubscriber(
          environment_controller::cDropPositionTopicName,
          lEnvironmentController);

  environment_controller::EmergencyStopSubscriber lEmergencyStopSubscriber =
      environment_controller::EmergencyStopSubscriber(
          environment_controller::cEmergencyStopTopicName,
          lEnvironmentController);

  environment_controller::ReleaseTimeSubscriber lReleaseTimeSubscriber =
      environment_controller::ReleaseTimeSubscriber(
          environment_controller::cReleaseTimeTopicName,
          lEnvironmentController);

  environment_controller::SensorSubscriber lSensorSubscriber =
      environment_controller::SensorSubscriber(
          environment_controller::cSensorTopicName, lEnvironmentController);

  franka_controller::FrankaFeedback lFrankaFeedback =
      franka_controller::FrankaFeedback(environment_controller::cFrankaFeedback,
                                        lEnvironmentController);

  ros::spin();

  return 0;
}
