#include "controller/CloseGripper.hpp"
#include "controller/Context.hpp"
#include "controller/EmergencyStop.hpp"
#include "controller/Init.hpp"
#include "controller/Move.hpp"
#include "controller/MoveToDropLocation.hpp"
#include "controller/Ready.hpp"
#include "controller/ReleaseCup.hpp"
#include "controller/TrajectoryProvider.hpp"
#include "controller/WaitForCup.hpp"
#include "controller/WaitForReleaseSignal.hpp"
#include "environment_controller/Cup.hpp"
#include "kinematics/Configuration.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
TEST(StateTransition, ReadyToEmergency)
{
  controller::Context* lContext = new controller::Context();
  lContext->hardStop(true);
  lContext->currentState()->doActivity(lContext);
  EXPECT_EQ(typeid(*lContext->currentState()),
            typeid(controller::EmergencyStop));
}

TEST(StateTransition, EmergencyStopToReady)
{
  controller::Context* lContext = new controller::Context();
  lContext->setState(std::make_shared<controller::EmergencyStop>());
  EXPECT_EQ(typeid(*lContext->currentState()),
            typeid(controller::EmergencyStop));
  lContext->hardStop(false);
  lContext->currentState()->doActivity(lContext);
  EXPECT_EQ(typeid(*lContext->currentState()), typeid(controller::Ready));
}

TEST(StateTransition, MoveToEmergencyStop)
{
  controller::Context* lContext = new controller::Context();
  environment_controller::Object lObject = environment_controller::Object(
      environment_controller::Position(0.2, 0.2, 0.2), 0.08, 0.08, 0.08, 0.0,
      0.0, ros::Time::now(), 0);
  environment_controller::Cup lCup =
      environment_controller::Cup(lObject, ros::Time::now());
  lContext->foundCup(lCup);
  lContext->currentState()->doActivity(lContext);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  EXPECT_EQ(typeid(*lContext->currentState()), typeid(controller::Move));

  lContext->hardStop(true);
  lContext->currentState()->doActivity(lContext);

  EXPECT_EQ(typeid(*lContext->currentState()),
            typeid(controller::EmergencyStop));
}

TEST(StateTransition, WaitForCupToGripper)
{
  controller::Context* lContext = new controller::Context();
  std::thread(&controller::Context::run, lContext).detach();
  environment_controller::Object lObject = environment_controller::Object(
      environment_controller::Position(0.2, 0.2, 0.2), 0.08, 0.08, 0.08, 0.0,
      0.0, ros::Time::now(), 0);
  environment_controller::Cup lCup = environment_controller::Cup(
      lObject, ros::Time::now() + ros::Duration(0, 10000000));
  lContext->cup() = lCup;
  lContext->setState(std::make_shared<controller::WaitForCup>());
  EXPECT_EQ(typeid(*lContext->currentState()), typeid(controller::WaitForCup));

  std::this_thread::sleep_for(std::chrono::milliseconds(4));
  EXPECT_EQ(typeid(*lContext->currentState()), typeid(controller::WaitForCup));
  std::this_thread::sleep_for(std::chrono::milliseconds(6));

  EXPECT_EQ(typeid(*lContext->currentState()),
            typeid(controller::CloseGripper));
  lContext->hardStop(true);
}

TEST(StateTransition, WaitForReleaseSignalToReleaseCup)
{
  controller::Context* lContext = new controller::Context();
  lContext->setState(std::make_shared<controller::WaitForReleaseSignal>());
  EXPECT_EQ(typeid(*lContext->currentState()),
            typeid(controller::WaitForReleaseSignal));
  lContext->provideReleaseTime(1);
  lContext->currentState()->doActivity(lContext);
  EXPECT_EQ(typeid(*lContext->currentState()), typeid(controller::ReleaseCup));
  lContext->hardStop(true);
}

TEST(StateTransition, ReadyToMove)
{
  controller::Context* lContext = new controller::Context();
  environment_controller::Object lObject = environment_controller::Object(
      environment_controller::Position(0.2, 0.2, 0.2), 0.08, 0.08, 0.08, 0.0,
      0.0, ros::Time::now(), 0);
  environment_controller::Cup lCup =
      environment_controller::Cup(lObject, ros::Time::now());
  EXPECT_EQ(typeid(*lContext->currentState()), typeid(controller::Ready));

  lContext->foundCup(lCup);
  lContext->currentState()->doActivity(lContext);
  EXPECT_EQ(typeid(*lContext->currentState()), typeid(controller::Move));
}