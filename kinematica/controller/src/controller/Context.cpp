#include "controller/Context.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/EmergencyStop.hpp"
#include "controller/Init.hpp"
#include "controller/Move.hpp"
#include "controller/PowerOff.hpp"
#include "controller/Ready.hpp"
#include "controller/SoftStop.hpp"
#include "environment_controller/Position.hpp"
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
        mGripperData(0.0, 0.0),
        mDropPosition(0.0, 0.0, 0.0),
        mPatientPosition(0.0, 0.0, 0.0),
        mReleaseTime_s(-1)
  {
    mGraph->addObstacle(cRobotObstacle);
    mGraph->addObstacle(cFloorObstacle);
    setState(std::make_shared<Init>());
    mCurrentState->doActivity(this);
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
    while (!(typeid(*mCurrentState) == typeid(Ready)) && ros::ok())
    {
      mHardStopMutex.unlock();
      mSoftStopMutex.unlock();
      mCurrentStateMutex.lock();
      mCurrentState->doActivity(this);
      mCurrentStateMutex.unlock();
      mHardStopMutex.lock();
      mSoftStopMutex.lock();
    }
    mHardStopMutex.unlock();
    mSoftStopMutex.unlock();
  }

  void Context::foundCup(const environment_controller::Cup& aCup)
  {
    mCup = aCup;
  }

  void Context::hardStop(bool aStop)
  {
    std::lock_guard<std::mutex> lHardLock(mHardStopMutex);
    mFeedbackDone.notify_all();
    mWaitForRelease.notify_all();
    std::lock_guard<std::mutex> lCurrentStateMutex(mCurrentStateMutex);
    if (aStop)
    {
      mHistoryState = mCurrentState;
      setState(std::make_shared<EmergencyStop>());
      ROS_WARN("STOP");
    }
    else
    {
      mCurrentState = mHistoryState;
      ROS_WARN("RELEASE");
    }
  }

  void Context::softStop(bool aStop)
  {
    std::lock_guard<std::mutex> lSoftLock(mSoftStopMutex);
    mFeedbackDone.notify_all();
    mWaitForRelease.notify_all();
    std::lock_guard<std::mutex> lCurrentStateMutex(mCurrentStateMutex);
    if (aStop)
    {
      mHistoryState = mCurrentState;
      setState(std::make_shared<SoftStop>());
      ROS_WARN("SOFTSTOP");
    }
    else
    {
      mCurrentState = mHistoryState;
      ROS_WARN("RELEASE");
    }
  }

  void Context::provideObstacles(const environment_controller::Obstacles&)
  {
  }

  void Context::frankaDone(bool aDone)
  {
    if (aDone)
    {
      std::unique_lock<std::mutex> lLock(mFeedbackMutex);
      mFeedbackDone.notify_all();
    }
  }

  void Context::provideReleaseTime(int16_t aReleaseTime)
  {
    std::unique_lock<std::mutex> lLock(mReleaseMutex);
    mReleaseTime_s = aReleaseTime;
    mWaitForRelease.notify_all();
  }

  void Context::providePatientPosition(
      const environment_controller::Position& aPosition)
  {
    mPatientPosition = aPosition;
  }

  void Context::provideDropPosition(
      const environment_controller::Position& aPosition)
  {
    mDropPosition = aPosition;
  }

  kinematics::Configuration& Context::currentConfiguration()
  {
    return mCurrentConfiguration;
  }

  kinematics::Configuration& Context::goalConfiguration()
  {
    return mGoalConfiguration;
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

  std::shared_ptr<State>& Context::currentState()
  {
    return mCurrentState;
  }

  environment_controller::Position& Context::dropPosition()
  {
    return mDropPosition;
  }

  environment_controller::Position& Context::patientPosition()
  {
    return mPatientPosition;
  }
  std::condition_variable& Context::waitForRelease()
  {
    return mWaitForRelease;
  }

  std::condition_variable& Context::feedbackDone()
  {
    return mFeedbackDone;
  }

  int16_t& Context::releaseTime_s()
  {
    return mReleaseTime_s;
  }

  std::mutex& Context::releaseMutex()
  {
    return mReleaseMutex;
  }

  std::mutex& Context::feedbackMutex()
  {
    return mFeedbackMutex;
  }
} // namespace controller