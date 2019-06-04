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

      Sensor lSensor(lPos, lRot, 0);

      provideSensor(lSensor);
    }
    catch (const std::exception& lE)
    {
      ROS_ERROR("%s", lE.what());
    }
  }

  void SensorSubscriber::provideSensor(const Sensor& aSensor)
  {
  }
} // namespace environment_controller