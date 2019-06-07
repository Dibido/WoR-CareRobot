/*
 *  Name: gazebo_transport_to_ros_topic.cpp
 *  Author: Joseph Coombe
 *  Date: 11/22/2017
 *  Edited: 11/27/2017
 *  Description:
 *   Subscribe to a Gazebo transport topic and publish to a ROS topic
 */

// Gazebo dependencies
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <iostream>
#include <math.h> /* sqrt */
#include <ros/ros.h>

// Received msg:
// force: 9352.04
// torque: 232.033
// Received msg:
// force: 4915.26
// torque: 350.487
// Received msg:
// force: 13687.9
// torque: 1238.97
// ^CReceived msg:
// force: 13205.5
// torque: 1616.44
// Received msg:
// force: 10035.4
// torque: 514.697

double pythagoras(double a, double b)
{
  return sqrt(pow(a, 2.0) + pow(b, 2.0));
}

double calculateLength(double x, double y, double z)
{
  return pythagoras(pythagoras(x, y), z);
}

void cb(const ConstWrenchStampedPtr& _msg)
{
  std::cout << "Received msg: " << std::endl;
  // std::cout << _msg->DebugString() << std::endl;
  // try WrenchStamped msgWrenchedStamped;
  // std::cout << ros::Time::now()            << std::endl;

  double force =
      calculateLength(_msg->wrench().force().x(), _msg->wrench().force().y(),
                      _msg->wrench().force().z());
  double torque =
      calculateLength(_msg->wrench().torque().x(), _msg->wrench().torque().y(),
                      _msg->wrench().torque().z());

  std::cout << "force: " << force << std::endl;
  std::cout << "torque: " << torque << std::endl;

  //   pub.publish(msgWrenchedStamped);
}

int main(int argc, char** argv)
{
  ROS_INFO("Starting gazebo_transport_to_ros_topic node");

  // Load Gazebo
  gazebo::client::setup(argc, argv);

  //   // Load ROS
  ros::init(argc, argv, "gazebo_transport_to_ros_topic");
  ros::NodeHandle n;

  // Create Gazebo node and init
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  std::string gazebo_transport_topic_to_sub =
      "/gazebo/default/Franka_panda/_joint5/force_torque/wrench";

  gazebo::transport::SubscriberPtr sub =
      node->Subscribe(gazebo_transport_topic_to_sub, cb);

  //   // Create ROS node and init

  // Listen to Gazebo force_torque sensor topic
  ros::Rate loop_rate(100); // 100 hz
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  gazebo::shutdown();
  return 0;
}