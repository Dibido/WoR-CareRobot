#include "controller/Context.hpp"
#include "ros/ros.h"
#include <memory>
#include <stdlib.h>
kinematics::EndEffector GetEndEffector();

int main(int argc, char** argv)
{
  const std::string cControlTopic = "robot_command";
  ros::init(argc, argv, cControlTopic);
  controller::Context ctx = controller::Context();
  ctx.run();

  // environment_controller::EnvironmentController lEnvironmentController =
  //     environment_controller::EnvironmentController();
  // environment_controller::SafetyController lSafetyController =
  //     environment_controller::SafetyController(
  //         std::make_shared<environment_controller::EnvironmentController>(
  //             lEnvironmentController));
  // environment_controller::ObstaclesSubscriber lObstacleSubscriber =
  //     environment_controller::ObstaclesSubscriber(
  //         std::make_shared<environment_controller::SafetyController>(
  //             lSafetyController),
  //         environment_controller::cObstacleTopicName);

  // ros::NodeHandle lControlNode;
  // ros::NodeHandle lStopNode;

  // ros::Rate lLoop_rate(environment_controller::cRate);

  // robotcontroller::RobotControlPublisher lRobotControlPub(
  //     lControlNode, cControlTopic, environment_controller::cQueue_size);

  // std::shared_ptr<kinematics::IConfigurationProvider> lConfigurationProvider
  // =
  //     std::make_shared<kinematics::ConfigurationProvider>();

  // kinematics::Configuration lCurrentConfiguration;

  // while (ros::ok())
  // {
  //   kinematics::EndEffector lGoalEndEffector = GetEndEffector();

  //   kinematics::Configuration lGoalConfiguration =
  //       lConfigurationProvider->inverseKinematics(lGoalEndEffector,
  //                                                 lCurrentConfiguration);
  //   ROS_INFO("Goal: [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f] \t result: %i",
  //            lGoalEndEffector.cX_m, lGoalEndEffector.cY_m,
  //            lGoalEndEffector.cZ_m, lGoalEndEffector.cRoll_rad,
  //            lGoalEndEffector.cPitch_rad, lGoalEndEffector.cYaw_rad,
  //            lGoalConfiguration.result());
  //   lRobotControlPub.publish(1.0, lGoalConfiguration);

  //   lCurrentConfiguration = lGoalConfiguration;
  //   ros::spinOnce();
  //   lLoop_rate.sleep();
  // }

  return 0;
}

kinematics::EndEffector GetEndEffector()
{
  static int8_t iterator = -1;
  ++iterator;
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