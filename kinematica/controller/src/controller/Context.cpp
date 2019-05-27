// Local
#include "controller/Context.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/EmergencyStop.hpp"
#include "controller/Init.hpp"
#include "controller/PowerOff.hpp"
#include "controller/Ready.hpp"
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
        mCurrentState(std::make_shared<PowerOff>()),
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
    setState(std::make_shared<Init>());
    mCurrentState->doActivity(this);
  }

  void Context::setState(const std::shared_ptr<State>& state)
  {
    if (mCurrentState)
    {
      mCurrentState->exitAction(this);
    }
    mCurrentState = state;

    mCurrentState->entryAction(this);
  }

  void Context::run()
  {
    mCurrentStateMutex.lock();
    mCurrentState->doActivity(this);
    mCurrentStateMutex.unlock();
    while ((typeid(*mCurrentState) != typeid(EmergencyStop) ||
            typeid(*mCurrentState) != typeid(Ready)) &&
           ros::ok())
    {
      mCurrentStateMutex.lock();
      mCurrentState->doActivity(this);
      mCurrentStateMutex.unlock();
    }
  }

  void Context::foundCup(const environment_controller::Cup& aCup)
  {
    mCup = aCup;
  }

  void Context::hardStop(bool aStop)
  {
    mCurrentStateMutex.lock();
    if (aStop)
      setState(std::make_shared<EmergencyStop>());
    else
      setState(std::make_shared<Init>());

    mCurrentStateMutex.unlock();
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