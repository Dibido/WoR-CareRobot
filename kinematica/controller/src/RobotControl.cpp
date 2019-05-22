#include "environment_controller/ObstacleSubsciber.hpp"
#include "kinematics/ConfigurationProvider.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
#include "ros/ros.h"

kinematics::EndEffector GetEndEffector();

int main(int argc, char** argv)
{
  const std::string cControlTopic = "robot_command";
  const uint16_t cQueue_size = 1000;
  const uint8_t cRate = 3;

  ros::init(argc, argv, cControlTopic);

  std::string lStr = std::string("/detectedObjects");
  environment_controller::ObstacleSubsciber subsriber(lStr);

  ros::NodeHandle lControlNode;
  ros::NodeHandle lStopNode;

  ros::Duration lLoop_rate(cRate);

  robotcontroller::RobotControlPublisher lRobotControlPub(
      lControlNode, cControlTopic, cQueue_size);

  std::shared_ptr<kinematics::IConfigurationProvider> lConfigurationProvider =
      std::make_shared<kinematics::ConfigurationProvider>();

  kinematics::Configuration lCurrentConfiguration;

  while (ros::ok())
  {
    kinematics::EndEffector lGoalEndEffector = GetEndEffector();

    kinematics::Configuration lGoalConfiguration =
        lConfigurationProvider->inverseKinematics(lGoalEndEffector,
                                                  lCurrentConfiguration);
    ROS_INFO("result: %i", lGoalConfiguration.result());
    lRobotControlPub.publish(1.0, lGoalConfiguration);

    lCurrentConfiguration = lGoalConfiguration;
    ros::spinOnce();
    lLoop_rate.sleep();
  }

  return 0;
}

kinematics::EndEffector GetEndEffector()
{
  static int8_t iterator = -1;
  ++iterator;
  ROS_INFO("GetEndEffector %d", iterator);
  switch (iterator)
  {
  case 1:
    return kinematics::EndEffector(0.18937, 0.294939, 0.737341, 1, -1.5, -M_PI);
  case 2:
    return kinematics::EndEffector(-0.224580, -0.21499, 0.993990, -2.925026,
                                   -0.679329, -2.146552);
  case 3:
    return kinematics::EndEffector(0.786246, 0.00, 0.276516, 0, 0, M_PI);
  default:
    iterator = 0;
    return kinematics::EndEffector(0.088, 0, 0.927, 0, 0, -M_PI);
  }
}