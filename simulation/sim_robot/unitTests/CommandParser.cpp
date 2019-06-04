
#include <ros/ros.h>
#define private public
#include <sim_robot/Command.hpp>
#include <sim_robot/CommandParser.hpp>
#undef private
// Bring in gtest
#include <gtest/gtest.h>
TEST(TestProtocol, parseCommandSpeedFactor0)
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
}
TEST(TestProtocol, parseCommandSpeedFactor1)
{
  commands::CommandParser parser;

  std::vector<commands::Command> global_out = {};
  std::vector<commands::Command> theta_out = {};
  std::vector<double> commandTheta = { 0, 0, 0, 0, 0, 0, 0 };
  jointVel_t speedFactor = { 1 };

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
}

TEST(TestProtocol, parseCommandTheta)
{
  commands::CommandParser parser;

  std::vector<commands::Command> global_out = {};
  std::vector<commands::Command> theta_out = {};
  std::vector<double> commandTheta = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
  jointVel_t speedFactor = { 0.0 };

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
TEST(TestProtocol, createCommandTheta)
{
  commands::CommandParser parser;

  std::vector<commands::Command> global_out = {};
  std::vector<commands::Command> theta_out = {};
  std::vector<double> commandTheta = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
  jointVel_t speedFactor = { 0.0 };

  for (uint16_t i = 0; i < 7; ++i)
  {
    commands::Command command(commands::eCommandType::MOVE,   // type
                              static_cast<jointChannel_t>(i), // channel
                              static_cast<jointRad_t>(commandTheta[i]), // rad
                              speedFactor); // speedFactor
    global_out.push_back(command);
  }
  parser.createCommandTheta(commandTheta, speedFactor, theta_out);
  EXPECT_EQ(global_out, theta_out);
}

TEST(TestProtocol, createStopCommandTheta)
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

  parser.createStopCommandTheta(true, out);
  EXPECT_EQ(global_out, out);
}
