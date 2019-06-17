#include "environment_controller/SensorSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

namespace environment_controller
{

  SensorSubscriber::SensorSubscriber(
      const std::string& aTopicName,
      const std::shared_ptr<EnvironmentController>& aController)
      : mSubscriber(mHandle.subscribe(aTopicName,
                                      cQueue_size,
                                      &SensorSubscriber::sensorCallback,
                                      this)),
        mEnvironmentController(aController)
  {
  }

  void SensorSubscriber::sensorCallback(
      const kinematica_msgs::SensorConstPtr& aMsg)
  {
    try
    {
      Position lPos(aMsg->pose.position.x, aMsg->pose.position.y,
                    aMsg->pose.position.z);
      Rotation lRot(aMsg->pose.orientation.x, aMsg->pose.orientation.y,
                    aMsg->pose.orientation.z, aMsg->pose.orientation.z);

      Pose lPose(lPos, lRot);

      Sensor lSensor(aMsg->sensorID, lPose);

      ROS_DEBUG("ADD sensor %i", lSensor.sensorID());
      provideSensor(lSensor);
    }
    catch (...)
    {
    }
  }

  void SensorSubscriber::provideSensor(const Sensor& aSensor)
  {
    mEnvironmentController->registerSensor(aSensor);
  }

} // namespace environment_controller