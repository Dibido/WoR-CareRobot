#include "robotcontroller/RobotControlPublisher.hpp"

#include "ros/ros.h"

Matrix<double, 6, 1> GetEndEffector();
std::vector<kinematics::Link> createConfiguration();

int main(int argc, char** argv)
{
  const std::string cControlTopic = "robot_command";
  const uint16_t cQueue_size = 1000;

  ros::init(argc, argv, cControlTopic);

  ros::NodeHandle lControlNode;
  ros::NodeHandle lStopNode;

  ros::Duration lMoveDuration(4);

  robotcontroller::RobotControlPublisher lRobotControlPub(
      lControlNode, cControlTopic, cQueue_size);

  kinematics::DenavitHartenberg lDen(createConfiguration());
  std::vector<double> lCurrentConfiguration = {
    0, 0, 0, 0, 0, 0, 0
  }; // Current configuration
  Matrix<double, 6, 1> lGoalEndEffector{
    0, 0, 0, 0, 0, 0
  }; // Determine with astar

  while (ros::ok())
  {
    lGoalEndEffector = GetEndEffector();
    std::vector<double> lGoalConfiguration =
        lDen.inverseKinematics(lGoalEndEffector, lCurrentConfiguration);
    lRobotControlPub.publish(1.0, lGoalConfiguration);
    lMoveDuration.sleep();
    lCurrentConfiguration = lGoalConfiguration;
  }

  return 0;
}

Matrix<double, 6, 1> GetEndEffector()
{
  static int8_t iterator = -1;
  ++iterator;
  ROS_INFO("GetEndEffector %d", iterator);
  switch (iterator)
  {
  case 1:
    return Matrix<double, 6, 1>{ 0.18937, 0.294939, 0.737341, 1, -1.5, -M_PI };
  case 2:
    return Matrix<double, 6, 1>{ -0.224580, -0.21499,  0.993990,
                                 -2.925026, -0.679329, -2.146552 };
  case 3:
    return Matrix<double, 6, 1>{
      0.786246, 0.00, 0.276516, 0,0,M_PI
    };
  default:
    iterator = 0;
    return Matrix<double, 6, 1>{ 0.088, 0, 0.927, 0, 0, -M_PI };
  }
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