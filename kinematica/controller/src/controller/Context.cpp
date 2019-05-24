// Local
#include "controller/Context.hpp"
#include "controller/Init.hpp"
#include "controller/PowerOff.hpp"
#include "controller/ControllerConsts.hpp"
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
            std::make_shared<kinematics::ConfigurationProvider>())
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

  std::shared_ptr<kinematics::ConfigurationProvider>& Context::configuration()
  {
    return mConfigurationProvider;
  }
} // namespace controller