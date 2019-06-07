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
    mTimer = mHandle.createTimer(ros::Duration(0.1),
                                 &SensorSubscriber::transformListen, this);
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

      provideSensor(lSensor);
    }
    catch (const std::exception& lE)
    {
      ROS_ERROR("%s", lE.what());
    }
  }

  void SensorSubscriber::provideSensor(const Sensor& aSensor)
  {
    mEnvironmentController->registerSensor(aSensor);
  }

  void SensorSubscriber::transformListen(const ros::TimerEvent&)
  {
    try
    {
      Pose lPose = mEnvironmentController->transformFrames(0);

      ROS_DEBUG("Transform x: %f, y: %f, z: %f; r: %f, p: %f, y: %f, q: %f",
                lPose.position().x_m(), lPose.position().y_m(),
                lPose.position().z_m(), lPose.rotation().roll_rad(),
                lPose.rotation().pitch_rad(), lPose.rotation().yaw_rad(),
                lPose.rotation().quaternion());
    }
    catch (tf2::TransformException& lEx)
    {
      ROS_WARN("%s", lEx.what());
    }
  }

} // namespace environment_controller