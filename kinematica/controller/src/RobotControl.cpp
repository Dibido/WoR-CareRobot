#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/ObstaclesSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"
#include "environment_controller/SafetyController.hpp"
#include "kinematics/ConfigurationProvider.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  const std::string cControlTopic = "robot_command";


  ros::init(argc, argv, cControlTopic);
  environment_controller::EnvironmentController lEnvironmentController =
      environment_controller::EnvironmentController();
  environment_controller::SafetyController lSafetyController =
      environment_controller::SafetyController(std::make_shared<environment_controller::EnvironmentController>(lEnvironmentController));
  environment_controller::ObstaclesSubscriber lObstacleSubscriber =
      environment_controller::ObstaclesSubscriber(std::make_shared<environment_controller::SafetyController>(lSafetyController),environment_controller::cObstacleTopicName);

  ros::NodeHandle lControlNode;
  ros::NodeHandle lStopNode;

  ros::Rate lLoop_rate(environment_controller::cRate);

  robotcontroller::RobotControlPublisher lRobotControlPub(
      lControlNode, cControlTopic, environment_controller::cQueue_size);

  std::shared_ptr<kinematics::IConfigurationProvider> lConfigurationProvider =
      std::make_shared<kinematics::ConfigurationProvider>();

  kinematics::Configuration lCurrentConfiguration;

  kinematics::EndEffector lGoalEndEffector(0, 0, 0, 0, 0, 0);

  kinematics::Configuration lGoalConfiguration =
      lConfigurationProvider->inverseKinematics(lGoalEndEffector,
                                                lCurrentConfiguration);

  while (ros::ok())
  {
    lRobotControlPub.publish(1.0, lGoalConfiguration);

    ros::spinOnce();
    lLoop_rate.sleep();
  }

  return 0;
}