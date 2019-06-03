#include "controller/TrajectoryProvider.hpp"
#include "kinematics/Configuration.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(ArrivalTime, singleJointChanged)
{
  ros::Time::init();
  kinematics::Configuration lCurrentConfiguration = kinematics::Configuration();
  lCurrentConfiguration.setTheta(0, 0);
  lCurrentConfiguration.setTheta(1, 10);
  lCurrentConfiguration.setTheta(2, 20);
  lCurrentConfiguration.setTheta(3, 30);
  lCurrentConfiguration.setTheta(4, 40);
  lCurrentConfiguration.setTheta(5, 50);
  lCurrentConfiguration.setTheta(6, 60);
  kinematics::Configuration lGoalConfiguration = lCurrentConfiguration;
  lGoalConfiguration.setTheta(6, 100);
  controller::TrajectoryProvider lTrajectoryProvider =
      controller::TrajectoryProvider();

  EXPECT_GT(lTrajectoryProvider
                .calculateArrivalTime(lCurrentConfiguration, lGoalConfiguration)
                ,
            ros::Time::now());
  EXPECT_LT(lTrajectoryProvider
                .calculateArrivalTime(lCurrentConfiguration, lGoalConfiguration), ros::Time::now()+ros::Duration(1));
}

TEST(ArrivalTime, multipleJointsChanged)
{
  ros::Time::init();
  kinematics::Configuration lCurrentConfiguration = kinematics::Configuration();
  lCurrentConfiguration.setTheta(0, 0);
  lCurrentConfiguration.setTheta(1, 10);
  lCurrentConfiguration.setTheta(2, 20);
  lCurrentConfiguration.setTheta(3, 30);
  lCurrentConfiguration.setTheta(4, 40);
  lCurrentConfiguration.setTheta(5, 50);
  lCurrentConfiguration.setTheta(6, 60);
  kinematics::Configuration lGoalConfiguration = lCurrentConfiguration;
    lGoalConfiguration.setTheta(5, 10);
  lGoalConfiguration.setTheta(6, 100);
  controller::TrajectoryProvider lTrajectoryProvider =
      controller::TrajectoryProvider();

  EXPECT_GT(lTrajectoryProvider
                .calculateArrivalTime(lCurrentConfiguration, lGoalConfiguration)
                ,
            ros::Time::now());
  EXPECT_LT(lTrajectoryProvider
                .calculateArrivalTime(lCurrentConfiguration, lGoalConfiguration), ros::Time::now()+ros::Duration(1));
}