// Local
#include "controller/Context.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Init.hpp"
#include "controller/PowerOff.hpp"
#include "environment_controller/Position.hpp"
// Libary
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <thread>
namespace controller
{
  Context::Context()
      : mGraph(std::make_shared<planning::Graph>()),
        mAstar(std::make_shared<planning::AStar>(mGraph)),
        mRobotControlPublisher(
            std::make_shared<robotcontroller::RobotControlPublisher>(
                mRobotControlHandle,
                cRobotCommandTopicName,
                cQueue_size)),
        mRobotGripperPublisher(
            std::make_shared<robotcontroller::RobotGripperPublisher>(
                mRobotGripperHandle,
                cRobotGripperTopicName,
                cQueue_size)),
        mRobotStopPublisher(
            std::make_shared<robotcontroller::RobotStopPublisher>(
                mRobotStopHandle,
                cRobotStopTopicName,
                cQueue_size)),
        mConfigurationProvider(
            std::make_shared<kinematics::ConfigurationProvider>()),
        mCurrentState(std::make_shared<Init>()),
        mCup(environment_controller::Cup(
            environment_controller::Object(
                environment_controller::Position(0.0, 0.0, 0.0),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                ros::Time(0),
                0),
            ros::Time(0))),
        mGripperData(0.0, 0.0)
  {
  }

  void Context::setState(const std::shared_ptr<State>& state)
  {
    // std::cout << __PRETTY_FUNCTION__ << std::endl;

    if (mCurrentState)
    {
      mCurrentState->exitAction(this);
    }
    mCurrentState = state;

    mCurrentState->entryAction(this);
  }

  void Context::run()
  {
    if (!mCurrentState)
    {
      setState(std::make_shared<Init>());
    }
    int i = 0;
    while (ros::ok() && typeid(*mCurrentState) != typeid(PowerOff))
    {
      mCurrentState->doActivity(this);
      if (++i > 10000)
      {
        setState(std::make_shared<PowerOff>());
      }
    }
  }

  void Context::foundCup(const environment_controller::Cup& aCup)
  {
    mCup = aCup;
  }

  void Context::hardStop(bool aStop)
  {
    mRobotStopPublisher->publish(aStop);
  }

  void Context::provideObstacles(
      const environment_controller::Obstacles& aObstacles)
  {
    for (const environment_controller::Obstacle& lObstacle : aObstacles)
    {
      // TODO environment_controller Obstacle to planning obstalce
    }
  }

  kinematics::Configuration& Context::configuration()
  {
    return mConfiguration;
  }

  std::shared_ptr<planning::Graph>& Context::graph()
  {
    return mGraph;
  }

  std::shared_ptr<planning::AStar>& Context::astar()
  {
    return mAstar;
  }

  std::shared_ptr<robotcontroller::RobotControlPublisher>&
      Context::robotControl()
  {
    return mRobotControlPublisher;
  }

  std::shared_ptr<robotcontroller::RobotGripperPublisher>&
      Context::robotGripper()
  {
    return mRobotGripperPublisher;
  }

  std::shared_ptr<robotcontroller::RobotStopPublisher>& Context::robotStop()
  {
    return mRobotStopPublisher;
  }

  std::shared_ptr<kinematics::ConfigurationProvider>&
      Context::configurationProvider()
  {
    return mConfigurationProvider;
  }

  environment_controller::Cup& Context::cup()
  {
    return mCup;
  }

  robotcontroller::GripperData& Context::gripperData()
  {
    return mGripperData;
  }
} // namespace controller