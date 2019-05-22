#include "environment_controller/ObstaclesSubscriber.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
#include "ros/ros.h"

std::vector<kinematics::Link> createConfiguration();

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

  kinematics::DenavitHartenberg lDen(createConfiguration());
  std::vector<double> lCurrentConfiguration = {
    0, 0, 0, 0, 0, 0, 0
  }; // Current configuration
  Matrix<double, 6, 1> lGoalEndEffector{
    0, 0, 0, 0, 0, 0
  }; // Determine with astar
  std::vector<double> lGoalConfiguration =
      lDen.inverseKinematics(lGoalEndEffector, lCurrentConfiguration);

  while (ros::ok())
  {
    lRobotControlPub.publish(1.0, lGoalConfiguration);

    ros::spinOnce();
    lLoop_rate.sleep();
  }

  return 0;
}

std::vector<kinematics::Link> createConfiguration()
{
  std::vector<kinematics::Link> joints;

  // The official config, uses the modified Denavit-Hartenberg
  // parameters Joint 1
  joints.push_back(kinematics::Link(0, 0, 0.333, kinematics::eJoint::REVOLUTE,
                                    kinematics::degree2Radian(-166),
                                    kinematics::degree2Radian(166)));
  // Joint 2
  joints.push_back(kinematics::Link(
      0, -M_PI_2, 0.0, kinematics::eJoint::REVOLUTE,
      kinematics::degree2Radian(-101), kinematics::degree2Radian(101)));
  // Joint 3
  joints.push_back(kinematics::Link(
      0.0, M_PI_2, 0.316, kinematics::eJoint::REVOLUTE,
      kinematics::degree2Radian(-166), kinematics::degree2Radian(166)));
  // Joint 4
  joints.push_back(kinematics::Link(
      0.0825, M_PI_2, 0.0, kinematics::eJoint::REVOLUTE,
      kinematics::degree2Radian(-176), kinematics::degree2Radian(-4)));
  // Joint 5
  joints.push_back(kinematics::Link(
      -0.0825, -M_PI_2, 0.384, kinematics::eJoint::REVOLUTE,
      kinematics::degree2Radian(-166), kinematics::degree2Radian(166)));
  // Joint 6
  joints.push_back(kinematics::Link(
      0.0, M_PI_2, 0.0, kinematics::eJoint::REVOLUTE,
      kinematics::degree2Radian(-1), kinematics::degree2Radian(215)));
  // Joint 7 -> This joint and the next have been combined to simplify
  // this prototype
  joints.push_back(kinematics::Link(
      0.088, M_PI_2, 0, kinematics::eJoint::REVOLUTE,
      kinematics::degree2Radian(-166), kinematics::degree2Radian(166)));
  // flange
  joints.push_back(
      kinematics::Link(0.0, 0.0, 0.107, 0.0, kinematics::eJoint::STATIC));
  return joints;
}