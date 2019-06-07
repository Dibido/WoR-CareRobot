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
    mTimer =
        mHandle.createTimer(ros::Duration(0.1), &SensorSubscriber::test, this);
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

      Sensor lSensor(0, lPose);

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

    /*
    Pose lPose = mEnvironmentController->transformFrames(0);

    std::cout << lPose.position().x_m() << " " << lPose.position().y_m() << " "
              << lPose.position().z_m() << std::endl;
    std::cout << lPose.rotation().roll_rad() << " "
              << lPose.rotation().pitch_rad() << " "
              << lPose.rotation().yaw_rad() << " "
              << lPose.rotation().quaternion() << std::endl;
              */
  }

  void SensorSubscriber::test(const ros::TimerEvent&)
  {
    // try
    // {
    // Pose lPose = mEnvironmentController->transformFrames(0);

    // std::cout << lPose.position().x_m() << " " << lPose.position().y_m()
    //           << " " << lPose.position().z_m() << std::endl;
    // std::cout << lPose.rotation().roll_rad() << " "
    //           << lPose.rotation().pitch_rad() << " "
    //           << lPose.rotation().yaw_rad() << " "
    //           << lPose.rotation().quaternion() << std::endl;
    // }
    // catch (...)
    // {
    //   std::cout << "shet";
    // }
  }

} // namespace environment_controller