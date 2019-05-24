#include "environment_controller/EnvironmentConsts.hpp"
#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/ObstaclesSubscriber.hpp"
#include "environment_controller/SafetyController.hpp"
#include "kinematics/ConfigurationProvider.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
#include "ros/ros.h"

kinematics::EndEffector GetEndEffector();

int main(int argc, char** argv)
{
  const std::string cControlTopic = "robot_command";

  ros::init(argc, argv, cControlTopic);
  environment_controller::EnvironmentController lEnvironmentController =
      environment_controller::EnvironmentController();
  environment_controller::SafetyController lSafetyController =
      environment_controller::SafetyController(
          std::make_shared<environment_controller::EnvironmentController>(
              lEnvironmentController));
  environment_controller::ObstaclesSubscriber lObstacleSubscriber =
      environment_controller::ObstaclesSubscriber(
          std::make_shared<environment_controller::SafetyController>(
              lSafetyController),
          environment_controller::cObstacleTopicName);

  ros::NodeHandle lControlNode;
  ros::NodeHandle lStopNode;

  ros::Duration lLoop(4);

  robotcontroller::RobotControlPublisher lRobotControlPub(
      lControlNode, cControlTopic, environment_controller::cQueue_size);

  std::shared_ptr<kinematics::IConfigurationProvider> lConfigurationProvider =
      std::make_shared<kinematics::ConfigurationProvider>();

  kinematics::Configuration lCurrentConfiguration;

  while (ros::ok())
  {
    kinematics::EndEffector lGoalEndEffector = GetEndEffector();

    kinematics::Configuration lGoalConfiguration =
        lConfigurationProvider->inverseKinematics(lGoalEndEffector,
                                                  lCurrentConfiguration);
    ROS_INFO("Goal: [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f] \t result: %i",
             lGoalEndEffector.cX_m, lGoalEndEffector.cY_m,
             lGoalEndEffector.cZ_m, lGoalEndEffector.cRoll_rad,
             lGoalEndEffector.cPitch_rad, lGoalEndEffector.cYaw_rad,
             lGoalConfiguration.result());
    lRobotControlPub.publish(1, lGoalConfiguration);

    lCurrentConfiguration = lGoalConfiguration;
    ros::spinOnce();
    lLoop.sleep();
  }

  return 0;
}

kinematics::EndEffector GetEndEffector()
{
  static int8_t iterator = -1;
  ++iterator;
  switch (iterator)
  {
  case 0:
    return kinematics::EndEffector(0.5, -0.2, 0.4, M_PI_2, M_PI_2 ,0 );
  case 1:
    return kinematics::EndEffector(0.5, -0.2, 0.5, M_PI_2, M_PI_2 ,0 );
  case 2:
    // return kinematics::EndEffector(0.5, -0.15, 0.1, M_PI_2, M_PI_2 ,0 );
  // case 3:
  //   return kinematics::EndEffector(0.3, -0.2, 0.6, M_PI_2, M_PI_2 ,0 );
  default:
    ROS_ASSERT_MSG(false, "No more configurations");
    // iterator = 0;
    // return kinematics::EndEffector(0.5, -0.2, 0.1, M_PI_2, M_PI_2 ,0 );
  }
}