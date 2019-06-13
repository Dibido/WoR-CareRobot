#include "controller/Context.hpp"
#include "controller/TrajectoryProvider.hpp"
#include "kinematics/Configuration.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(ArrivalTime, singleJointChanged)
{
  controller::Context* lContext = new controller::Context();
  lContext->currentConfiguration().setTheta(0, 0);
  lContext->currentConfiguration().setTheta(1, 10);
  lContext->currentConfiguration().setTheta(2, 20);
  lContext->currentConfiguration().setTheta(3, 30);
  lContext->currentConfiguration().setTheta(4, 40);
  lContext->currentConfiguration().setTheta(5, 50);
  lContext->currentConfiguration().setTheta(6, 60);
  kinematics::Configuration lGoalConfiguration = lContext->goalConfiguration();
  lGoalConfiguration.setTheta(6, 100);
  controller::TrajectoryProvider lTrajectoryProvider =
      controller::TrajectoryProvider();

  EXPECT_GT(
      lTrajectoryProvider.calculateArrivalTime(lContext, lGoalConfiguration),
      ros::Time::now());
  EXPECT_LT(
      lTrajectoryProvider.calculateArrivalTime(lContext, lGoalConfiguration),
      ros::Time::now() + ros::Duration(2));
}

TEST(ArrivalTime, multipleJointsChanged)
{
  controller::Context* lContext = new controller::Context();
  lContext->currentConfiguration().setTheta(0, 0);
  lContext->currentConfiguration().setTheta(1, 10);
  lContext->currentConfiguration().setTheta(2, 20);
  lContext->currentConfiguration().setTheta(3, 30);
  lContext->currentConfiguration().setTheta(4, 40);
  lContext->currentConfiguration().setTheta(5, 50);
  lContext->currentConfiguration().setTheta(6, 60);
  kinematics::Configuration lGoalConfiguration = lContext->goalConfiguration();
  lGoalConfiguration.setTheta(5, 10);
  lGoalConfiguration.setTheta(6, 100);
  controller::TrajectoryProvider lTrajectoryProvider =
      controller::TrajectoryProvider();

  EXPECT_GT(
      lTrajectoryProvider.calculateArrivalTime(lContext, lGoalConfiguration),
      ros::Time::now());
  EXPECT_LT(
      lTrajectoryProvider.calculateArrivalTime(lContext, lGoalConfiguration),
      ros::Time::now() + ros::Duration(2));
}
