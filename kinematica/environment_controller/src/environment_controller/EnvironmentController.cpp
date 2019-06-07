#include "environment_controller/EnvironmentController.hpp"
#include "controller/EmergencyStop.hpp"
#include "environment_controller/SafetyController.hpp"
#include "environment_controller/TFHandler.hpp"

#include <thread>

namespace environment_controller
{
  EnvironmentController::EnvironmentController(
      const std::shared_ptr<controller::Context>& aContext)
      : mContext(aContext), mTfHandler(std::make_shared<TFHandler>())
  {
    mTimer = mCallbackNode.createTimer(ros::Duration(cSensorTFPublishRate),
                                       &EnvironmentController::publishTFSensors,
                                       this);
  }

  void EnvironmentController::provideObstacles(const Obstacles& aObstacles)
  {
    mContext->provideObstacles(aObstacles);
  }

  void EnvironmentController::executeHardstop(bool hardstop)
  {
    if (typeid(*mContext->currentState()) ==
            typeid(controller::EmergencyStop) ||
        hardstop == true)
      mContext->hardStop(hardstop);
  }

  void EnvironmentController::provideCup(const Cup& aCup)
  {
    mContext->foundCup(aCup);
    std::thread(&controller::Context::run, mContext).detach();
  }

  void EnvironmentController::provideGoal(const Position& aPosition)
  {
    mContext->provideDropPosition(aPosition);
  }

  void EnvironmentController::provideReleaseTime(const uint8_t aReleaseTime_s)
  {
    mContext->provideReleaseTime(aReleaseTime_s);
  }

  void EnvironmentController::registerSensor(const Sensor& aSensor)
  {
    mSensors.insert(std::make_pair(aSensor.sensorID(), aSensor.pose()));
  }

  Pose EnvironmentController::transformFrames(const uint8_t aSensorID)
  {
    Pose lPose = mTfHandler->calculatePosition(
        std::string(cSensorFrame) + std::to_string(aSensorID), cGlobalFrame);
    return lPose;
  }

  void EnvironmentController::publishTFSensors(const ros::TimerEvent&)
  {
    for (auto lSensor = mSensors.begin(); lSensor != mSensors.end(); ++lSensor)
    {
      mTfHandler->transform(lSensor->second,
                            std::string(cSensorFrame) +
                                std::to_string(lSensor->first));
    }
  }

  const Sensor EnvironmentController::getSensor(const uint8_t aSensorID) const
  {
    Sensor lSensor(mSensors.find(aSensorID)->first,
                   mSensors.find(aSensorID)->second);

    return lSensor;
  }
} // namespace environment_controller