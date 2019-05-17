#include "robotcontroller/RobotControlPublisher.hpp"

#include <ros/ros.h>

namespace robotcontroller {

  RobotControlPublisher::RobotControlPublisher(ros::NodeHandle& lN) : mN(lN) 
  {

  }

  void RobotControlPublisher::publish(const double lSf, const std::vector<kinematics::Link>& joints)
  {

  ros::Publisher lRobotcontrol_pub =
      mN.advertise<robotcontroller_msgs::Control>("robot_command", 1000);

  ros::Rate lLoop_rate(10);

  kinematics::DenavitHartenberg lDen(joints);
  std::vector<double> lCurrentConfiguration = {
    0, 0, 0, 0, 0, 0, 0
  }; // Current configuration
  Matrix<double, 6, 1> lGoalEndEffector{
    0, 0, 0, 0, 0, 0
  }; // Determine with astar
  std::vector<double> lGoalConfiguration =
      lDen.inverseKinematics(lGoalEndEffector, lCurrentConfiguration);

    robotcontroller_msgs::Control lMsg;

    lMsg.theta = lGoalConfiguration;
    lMsg.sf = lSf;

    lRobotcontrol_pub.publish(lMsg);

    lCurrentConfiguration = lGoalConfiguration;

    ros::spinOnce();

    lLoop_rate.sleep();
  }

}