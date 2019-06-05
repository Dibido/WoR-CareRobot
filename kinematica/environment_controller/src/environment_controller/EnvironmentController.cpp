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
    mSensors.at(aSensor.sensorID()) = aSensor.pose();
    tfHandler->transform(aSensor.pose(),
                         cSensorFrame + std::to_string(aSensor.sensorID()));
  }

  const Sensor& EnvironmentController::getSensor(const uint8_t aSensorID) const
  {
    Sensor lSensor(mSensors.find(aSensorID)->first,
                   mSensors.find(aSensorID)->second);

    return lSensor;
  }
} // namespace environment_controller