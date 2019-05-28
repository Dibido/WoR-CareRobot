#include "controller/Move.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Init.hpp"
#include "controller/ReleaseCup.hpp"
#include "controller/WaitForCup.hpp"
#include "kinematics/EndEffector.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
namespace controller
{
  Move::Move(){

  };
  Move::~Move(){};

  void Move::entryAction(Context* aContext)
  {
    auto start = ros::Time::now();
    ROS_INFO("Start Move calculation");
    kinematics::EndEffector lEndEffector = kinematics::EndEffector(
        aContext->cup().object().position().x_m(),
        aContext->cup().object().position().y_m(),
        aContext->cup().object().position().z_m(), 0, M_PI_2, M_PI_2);

    kinematics::Configuration lConfiguration =
        aContext->configurationProvider()->inverseKinematics(
            lEndEffector, aContext->configuration());

    if (lConfiguration.result() == true)
    {
      planning::Path requiredPath = findPath(aContext, lEndEffector);
      lConfiguration = aContext->configuration();
      ROS_INFO("Found path, size: %i", requiredPath.size());

      for (std::size_t i = 1; i < requiredPath.size(); ++i)
      {
        kinematics::EndEffector lTrajectoryEndEffector =
            kinematics::EndEffector(
                static_cast<double>(requiredPath[i].x) / 100,
                static_cast<double>(requiredPath[i].y) / 100,
                static_cast<double>(requiredPath[i].z) / 100, 0, M_PI_2,
                M_PI_2);

        lConfiguration = aContext->configurationProvider()->inverseKinematics(
            lTrajectoryEndEffector, lConfiguration);

        if (lConfiguration.result() == false)
        {
          ROS_ERROR("Could not find configuration for trajectory %i", i);
          aContext->setState(std::make_shared<Init>());
          return;
        }
        mTrajectory.push(lConfiguration);
      }
      lConfiguration = aContext->configurationProvider()->inverseKinematics(
          lEndEffector, lConfiguration);

      if (lConfiguration.result() == false)
      {
        aContext->setState(std::make_shared<Init>());
        return;
      }
      mTrajectory.push(lConfiguration);
    }
    else
    {
      ROS_ERROR("Unreachable");
      // Target end effector is not reachable
      aContext->setState(std::make_shared<Init>());
      return;
    }

    mArrivalTime = ros::Time::now();
    ros::Duration dur = ros::Time::now() - start;
    ROS_INFO("End Move calculation %f", ( double )dur.toNSec() / 1000000000);
  }

  void Move::doActivity(Context* aContext)
  {
    if (mArrivalTime.sleepUntil(mArrivalTime))
    {
      if (mTrajectory.size() == 0)
      {
        aContext->setState(std::make_shared<WaitForCup>());
        return;
      }
      else
      {
        kinematics::Configuration& lTargetConfiguration = mTrajectory.front();
        aContext->robotControl()->publish(cSpeedFactor, lTargetConfiguration);
        mArrivalTime = calculateArrivalTime(aContext, lTargetConfiguration);
        aContext->configuration() = lTargetConfiguration;
        ROS_INFO(
            "Move to \n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f",
            lTargetConfiguration[0], lTargetConfiguration[1],
            lTargetConfiguration[2], lTargetConfiguration[3],
            lTargetConfiguration[4], lTargetConfiguration[5],
            lTargetConfiguration[6]);
        mTrajectory.pop();
      }
    }
  }

  void Move::exitAction(Context*)
  {
  }

  ros::Time Move::calculateArrivalTime(Context* aContext,
                                       kinematics::Configuration lConfiguration)
  {
    double lMaxDeltaTheta = 0;
    for (size_t i = 0; i < lConfiguration.size; ++i)
    {
      if (lMaxDeltaTheta <
          std::abs(lConfiguration[i] - aContext->configuration()[i]))
      {
        lMaxDeltaTheta =
            std::abs(lConfiguration[i] - aContext->configuration()[i]);
      }
    }

    return ros::Time::now() +
           ros::Duration(lMaxDeltaTheta / cJointSpeed_rads / cSpeedFactor);
  }

  planning::Path Move::findPath(Context* aContext,
                                const kinematics::EndEffector& aGoal)
  {
    kinematics::EndEffector lEndEffector =
        aContext->configurationProvider()->forwardKinematics(
            aContext->configuration());

    planning::Vertex lStart(static_cast<long>(lEndEffector.cX_m * 100),
                            static_cast<long>(lEndEffector.cY_m * 100),
                            static_cast<long>(lEndEffector.cZ_m * 100));

    planning::Vertex lGoal(static_cast<long>(aGoal.cX_m * 100),
                           static_cast<long>(aGoal.cY_m * 100),
                           static_cast<long>(aGoal.cZ_m * 100));

    return aContext->astar()->search(lStart, lGoal);
  }
} // namespace controller