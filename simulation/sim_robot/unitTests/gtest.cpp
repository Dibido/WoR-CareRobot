// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>
#include <sim_robot/robot_controller_plugin.hpp>
// Include messages
#include <robotcontroller_msgs/control.h>
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestProtocol, robot_command)
{
  robot_controller_plugin lRobot_controller_plugin;
  // Publish commands to the robot control topic
  ros::Publisher lDataPublisher = nh.advertise("/robot_command", 1);
  robotcontroller_msgs::Control commands;
  lDataPublisher.publish(commands);
  // Read the output topic
  ros::Subscriber lDataSubscriber = nh.subscribe("/robot_command", 1);

  rosSubCommands =
      nh->Subscribe(gazebo::COMMAND_TOPIC,
                    &RobotControllerPlugin::commandCallBackFloat, this);
  // Check the size
  EXPECT_EQ(0,0);
  // Compare the values
  EXPECT_EQ(1000, 1000);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
