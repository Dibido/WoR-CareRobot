#include "environment_controller/ObstaclesSubscriber.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  const std::string cControlTopic = "robot_command";
  const uint16_t cQueue_size = 1000;
  const uint8_t cRate = 10;

  ros::init(argc, argv, cControlTopic);

  std::string lStr = std::string("/detectedObjects");
  environment_controller::ObstaclesSubscriber lSubscriber(lStr);

  ros::NodeHandle lControlNode;
  ros::NodeHandle lStopNode;

  ros::Rate lLoop_rate(cRate);

  robotcontroller::RobotControlPublisher lRobotControlPub(
      lControlNode, cControlTopic, cQueue_size);

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