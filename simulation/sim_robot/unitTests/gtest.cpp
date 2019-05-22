// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>
#include <sim_robot/Command.hpp>
#include <sim_robot/CommandParser.hpp>
// Include messages
#include <robotcontroller_msgs/Control.h>
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestProtocol, parseCommandThetaSpeedFactor)
{
  commands::CommandParser parser;

  std::vector<commands::Command> global_out = {};
  std::vector<commands::Command> theta_out = {};
  std::vector<double> commandTheta = { 0, 0, 0, 0, 0, 0, 0 };
  jointVel_t speedFactor = { 0 };

  for (uint16_t i = 0; i < 7; ++i)
  {
    commands::Command command(commands::eCommandType::MOVE,   // type
                              static_cast<jointChannel_t>(i), // channel
                              static_cast<jointRad_t>(commandTheta[i]), // rad
                              speedFactor); // speedFactor
    global_out.push_back(command);
  }

  parser.parseCommandTheta(commandTheta, speedFactor, theta_out);
  EXPECT_EQ(global_out, theta_out);

  commandTheta = { 0, 0, 0, 0, 0, 0, 0 };
  speedFactor = { 1 };

  for (uint16_t i = 0; i < 7; ++i)
  {
    commands::Command command(commands::eCommandType::MOVE,   // type
                              static_cast<jointChannel_t>(i), // channel
                              static_cast<jointRad_t>(commandTheta[i]), // rad
                              speedFactor); // speedFactor
    global_out.push_back(command);
  }

  parser.parseCommandTheta(commandTheta, speedFactor, theta_out);
  EXPECT_NE(global_out, theta_out);
}
TEST(TestProtocol, parseCommandTheta)
{
  commands::CommandParser parser;

  std::vector<commands::Command> global_out = {};
  std::vector<commands::Command> theta_out = {};
  std::vector<double> commandTheta = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  jointVel_t speedFactor = { 0 };

  for (uint16_t i = 0; i < 7; ++i)
  {
    commands::Command command(commands::eCommandType::MOVE,   // type
                              static_cast<jointChannel_t>(i), // channel
                              static_cast<jointRad_t>(commandTheta[i]), // rad
                              speedFactor); // speedFactor
    global_out.push_back(command);
  }

  
}

TEST(TestProtocol, parseCommandStop)
{
  std::vector<commands::Command> global_out = {};
  commands::CommandParser parser;
  std::vector<commands::Command> out;

  parser.parseCommandStop(false, out);
  EXPECT_EQ(global_out, out);

  for (uint16_t i = 0; i < 7; ++i)
  {
    commands::Command command(commands::eCommandType::STOP,   // type
                              static_cast<jointChannel_t>(i), // channel
                              static_cast<jointRad_t>(0),     // rad
                              0);                             // speedFactor
    global_out.push_back(command);
  }

  parser.parseCommandStop(true, out);
  EXPECT_EQ(global_out, out);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
