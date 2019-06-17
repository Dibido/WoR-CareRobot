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
    mTimer = mHandle.createTimer(ros::Duration(cSensorCallbackDuration_s),
                                 &EnvironmentController::transformDebug, this);
  }

  void EnvironmentController::provideObstacles(const Obstacles& aObstacles)
  {
    mContext->provideObstacles(aObstacles);
  }

  void EnvironmentController::executeHardstop(bool hardstop)
  {
    if (typeid(*mContext->currentState()) ==
            typeid(controller::EmergencyStop) &&
        hardstop == false)
    {
      mContext->hardStop(hardstop);
    }
    else if (hardstop == true && typeid(*mContext->currentState()) !=
                                     typeid(controller::EmergencyStop))
    {
      mContext->hardStop(hardstop);
    }
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
    mTfHandler->transform(aSensor.pose(), true, cGlobalFrame,
                          std::string(cSensorFrame) +
                              std::to_string(aSensor.sensorID()));
  }

  Pose EnvironmentController::transformFrames(const std::string& aFrame,
                                              const uint8_t aID)
  {
    Pose lPose = mTfHandler->calculatePosition(
        std::string(aFrame) + std::to_string(aID), cGlobalFrame);
    return lPose;
  }

  void EnvironmentController::setObstacles(const Obstacles& aObstacles)
  {
    mObstacles.clear();
    mObstacles = aObstacles;
    Rotation lRotationTemp(0.0, 0.0, 0.0, 1.0);

    for (uint8_t i = 0; i < aObstacles.size(); ++i)
    {
      Pose lPose(aObstacles.at(i).position(), lRotationTemp);
      mTfHandler->transform(lPose, false,
                            std::string(cSensorFrame) +
                                std::to_string(aObstacles.at(i).sensorId()),
                            std::string(cObstacleFrame) + std::to_string(i));
    }
  }

  const Sensor EnvironmentController::getSensor(const uint8_t aSensorID) const
  {
    std::map<uint8_t, Pose>::const_iterator lSensorIterator =
        mSensors.find(aSensorID);

    Sensor lSensor(lSensorIterator->first, lSensorIterator->second);

    return lSensor;
  }

  void EnvironmentController::transformDebug(const ros::TimerEvent&)
  {
    for (uint8_t i = 0; i < mObstacles.size(); ++i)
    {
      try
      {
        Pose lPose = transformFrames(cObstacleFrame, i);

        ROS_DEBUG("Transform x: %f, y: %f, z: %f; x: %f, y: %f, z: %f, w: %f",
                  lPose.position().x_m(), lPose.position().y_m(),
                  lPose.position().z_m(), lPose.rotation().x(),
                  lPose.rotation().y(), lPose.rotation().z(),
                  lPose.rotation().w());
      }
      catch (tf2::TransformException& lEx)
      {
        ROS_WARN("%s", lEx.what());
      }
    }
  }
} // namespace environment_controller