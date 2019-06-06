#include "environment_controller/EnvironmentController.hpp"
#include "controller/EmergencyStop.hpp"
#include "environment_controller/SafetyController.hpp"
#include "environment_controller/TFHandler.hpp"

#include <thread>
namespace environment_controller
{
  EnvironmentController::EnvironmentController(
      const std::shared_ptr<controller::Context>& aContext)
      : mContext(aContext), tfHandler(std::make_shared<TFHandler>())
  {
    // ros::Rate(cCallbackRate);
    mTimer = mCallbackNode.createTimer(
        ros::Duration(0.01), &EnvironmentController::publishTFSensors, this);
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
    std::string s = std::string(cSensorFrame) + std::to_string(aSensorID);
    std::cout << s << std::endl;
    Pose lPose = tfHandler->calculatePosition(s, cGlobalFrame);
    return lPose;
  }

  void EnvironmentController::publishTFSensors(const ros::TimerEvent&)
  {
    for (auto lSensor = mSensors.begin(); lSensor != mSensors.end(); ++lSensor)
    {
      tfHandler->transform(lSensor->second,
                           cSensorFrame + std::to_string(lSensor->first));
    }
  }

  const Sensor EnvironmentController::getSensor(const uint8_t aSensorID) const
  {
    Sensor lSensor(mSensors.find(aSensorID)->first,
                   mSensors.find(aSensorID)->second);

    return lSensor;
  }
} // namespace environment_controller