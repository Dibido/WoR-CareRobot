#include "controller/Context.hpp"
#include "controller/TrajectoryProvider.hpp"
#include "kinematics/Configuration.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(ArrivalTime, singleJointChanged)
{
  ros::Time::init();
  controller::Context* lContext = new controller::Context();
  lContext->configuration().setTheta(0, 0);
  lContext->configuration().setTheta(1, 10);
  lContext->configuration().setTheta(2, 20);
  lContext->configuration().setTheta(3, 30);
  lContext->configuration().setTheta(4, 40);
  lContext->configuration().setTheta(5, 50);
  lContext->configuration().setTheta(6, 60);
  kinematics::Configuration lGoalConfiguration = lContext->configuration();
  lGoalConfiguration.setTheta(6, 100);
  controller::TrajectoryProvider lTrajectoryProvider =
      controller::TrajectoryProvider();

  EXPECT_GT(
      lTrajectoryProvider.calculateArrivalTime(lContext, lGoalConfiguration),
      ros::Time::now());
  EXPECT_LT(
      lTrajectoryProvider.calculateArrivalTime(lContext, lGoalConfiguration),
      ros::Time::now() + ros::Duration(1));
}

TEST(ArrivalTime, multipleJointsChanged)
{
  ros::Time::init();
  controller::Context* lContext = new controller::Context();
  lContext->configuration().setTheta(0, 0);
  lContext->configuration().setTheta(1, 10);
  lContext->configuration().setTheta(2, 20);
  lContext->configuration().setTheta(3, 30);
  lContext->configuration().setTheta(4, 40);
  lContext->configuration().setTheta(5, 50);
  lContext->configuration().setTheta(6, 60);
  kinematics::Configuration lGoalConfiguration = lContext->configuration();
  lGoalConfiguration.setTheta(5, 10);
  lGoalConfiguration.setTheta(6, 100);
  controller::TrajectoryProvider lTrajectoryProvider =
      controller::TrajectoryProvider();

  EXPECT_GT(
      lTrajectoryProvider.calculateArrivalTime(lContext, lGoalConfiguration),
      ros::Time::now());
  EXPECT_LT(
      lTrajectoryProvider.calculateArrivalTime(lContext, lGoalConfiguration),
      ros::Time::now() + ros::Duration(1));
}

