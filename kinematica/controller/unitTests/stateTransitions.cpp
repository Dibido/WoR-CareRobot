#include "controller/Context.hpp"
#include "controller/EmergencyStop.hpp"
#include "controller/TrajectoryProvider.hpp"
#include "kinematics/Configuration.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
TEST(EmergencyStopTransition, enterEmergencyStopFromReady)
{
  controller::Context* lContext = new controller::Context();
  std::thread(&controller::Context::run, lContext).detach();
  lContext->hardStop(true);
  EXPECT_EQ(typeid(*lContext->currentState()),
            typeid(controller::EmergencyStop));
}

TEST(EmergencyStopTransition, enterEmergencyStopFromMove)
{
  controller::Context* lContext = new controller::Context();
  std::thread(&controller::Context::run, lContext).detach();
  lContext->hardStop(true);
  EXPECT_EQ(typeid(*lContext->currentState()),
            typeid(controller::EmergencyStop));
}