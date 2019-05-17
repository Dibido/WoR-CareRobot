#include "robotcontroller/RobotControlPublisher.hpp"
#include "ros/ros.h"

std::vector<kinematics::Link> createConfiguration();

int main(int argc, char** argv)
{

  ros::init(argc, argv, "robot_command");

  ros::NodeHandle lN;

  robotcontroller::RobotControlPublisher lRobotControlPub(lN);

  while (ros::ok())
  {
    lRobotControlPub.publish(1.0, createConfiguration());
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