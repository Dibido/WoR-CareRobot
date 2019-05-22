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
}
TEST(operator, equalsNotOperator)
{
  commands::Command command1(commands::eCommandType::STOP,
                             static_cast<jointChannel_t>(0), // channel
                             static_cast<jointRad_t>(0),     // rad
                             0);

  commands::Command command2(commands::eCommandType::MOVE,
                             static_cast<jointChannel_t>(0), // channel
                             static_cast<jointRad_t>(0),     // rad
                             0);
  EXPECT_NE(command1, command2);
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
}

