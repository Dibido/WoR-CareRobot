#include <ros/ros.h>
#include <sim_robot/Command.hpp>

// Include messages
// Bring in gtest
#include <gtest/gtest.h>

TEST(operator, equalsOperator)
{
  commands::Command command1(commands::eCommandType::STOP,
                             static_cast<jointChannel_t>(0), // channel
                             static_cast<jointRad_t>(0),     // rad
                             0);

  commands::Command command2(commands::eCommandType::STOP,
                             static_cast<jointChannel_t>(0), // channel
                             static_cast<jointRad_t>(0),     // rad
                             0);
  EXPECT_EQ(command1, command2);
  EXPECT_EQ(command1.getRad(), command2.getRad());
  EXPECT_EQ(command1.getChannel(), command2.getChannel());
  EXPECT_EQ(command1.getSpeedFactor(), command2.getSpeedFactor());
  EXPECT_EQ(command1.getTime(), command2.getTime());
}
TEST(operator, equalsNotOperator)
{
  commands::Command command1(commands::eCommandType::STOP,
                             static_cast<jointChannel_t>(0), // channel
                             static_cast<jointRad_t>(0),     // rad
                             0);

  commands::Command command2(commands::eCommandType::MOVE,
                             static_cast<jointChannel_t>(1), // channel
                             static_cast<jointRad_t>(1),     // rad
                             1);
  EXPECT_NE(command1, command2);
  EXPECT_NE(command1.getRad(), command2.getRad());
  EXPECT_NE(command1.getChannel(), command2.getChannel());
  EXPECT_NE(command1.getSpeedFactor(), command2.getSpeedFactor());
}
TEST(operator, assigmentOperator)
{
  commands::Command command1(commands::eCommandType::STOP,
                             static_cast<jointChannel_t>(0), // channel
                             static_cast<jointRad_t>(0),     // rad
                             0);

  commands::Command command2(commands::eCommandType::MOVE,
                             static_cast<jointChannel_t>(0), // channel
                             static_cast<jointRad_t>(0),     // rad
                             0);
  command2 = command1;

  EXPECT_EQ(command1, command2);
  EXPECT_EQ(command1.getRad(), command2.getRad());
  EXPECT_EQ(command1.getChannel(), command2.getChannel());
  EXPECT_EQ(command1.getSpeedFactor(), command2.getSpeedFactor());
  EXPECT_EQ(command1.getTime(), command2.getTime());
}
