#include "kinematics/DenavitHartenberg.hpp"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");

  std::vector<kinematics::Link> joints;

  // The official config, uses the modified Denavit-Hartenberg
  // parameters Joint 1
  joints.push_back(kinematics::Link(0, 0, 0.333, kinematics::Joint::REVOLUTE,
                                    -DEG_180, DEG_180));
  // Joint 2
  joints.push_back(kinematics::Link(
      0, -DEG_90, 0.0, kinematics::Joint::REVOLUTE, -DEG_180, DEG_180));
  // Joint 3
  joints.push_back(kinematics::Link(
      0.0, DEG_90, 0.316, kinematics::Joint::REVOLUTE, -DEG_180, DEG_180));
  // Joint 4
  joints.push_back(kinematics::Link(
      0.0825, DEG_90, 0.0, kinematics::Joint::REVOLUTE, -DEG_180, DEG_180));
  // Joint 5
  joints.push_back(kinematics::Link(
      -0.0825, -DEG_90, 0.384, kinematics::Joint::REVOLUTE, -DEG_180, DEG_180));
  // Joint 6
  joints.push_back(kinematics::Link(
      0.0, DEG_90, 0.0, kinematics::Joint::REVOLUTE, -DEG_180, DEG_180));
  // Joint 7
  joints.push_back(kinematics::Link(
      0.088, DEG_90, 0.0, kinematics::Joint::REVOLUTE, -DEG_180, DEG_180));
  // flange
  joints.push_back(
      kinematics::Link(0.0, 0.0, 0.102, 0.0, kinematics::Joint::STATIC));

  kinematics::DenavitHartenberg den(joints);
  std::vector<double> variables = { 0, 0, 0, 0, 0, 0, 0, 0 };

  std::cout << "All variables 0, official Franka Panda config" << std::endl;
  std::cout << den.forwardKinematicsYPR(variables) << std::endl;

  std::vector<kinematics::Link> jointsSimplify;

  jointsSimplify.push_back(kinematics::Link(
      1, 0, 2, kinematics::Joint::REVOLUTE, -DEG_180, DEG_180));
  jointsSimplify.push_back(kinematics::Link(
      1, DEG_90, 2, kinematics::Joint::REVOLUTE, -DEG_180, DEG_180));
  variables = { DEG_45, DEG_45 };

  kinematics::DenavitHartenberg denSimplify(jointsSimplify);
  std::cout << "Alpha = 0, Theta = 45" << std::endl;
  std::cout << denSimplify.forwardKinematicsYPR(variables, 0, 1) << std::endl;
  std::cout << "Alpha = 90, Theta = 45" << std::endl;
  std::cout << denSimplify.forwardKinematicsYPR(variables, 1, 2) << std::endl;

  return 0;
}
