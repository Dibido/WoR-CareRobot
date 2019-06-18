#include "lidar_application/SensorPublisher.hpp"

namespace lidar_application
{

  SensorPublisher::SensorPublisher(ros::NodeHandle& aHandle,
                                   const std::string& aTopic,
                                   const uint16_t aQueue_size)
      : mHandle(aHandle),
        cTopic(aTopic),
        cQueue_size(aQueue_size),
        mSensor_pub(
            mHandle.advertise<kinematica_msgs::Sensor>(cTopic, cQueue_size))
  {
  }

  void SensorPublisher::provideSensor(
      const environment_controller::Sensor& aSensor)
  {
    kinematica_msgs::Sensor lMsg;

    lMsg.pose.position.x = aSensor.pose().position().x_m();
    lMsg.pose.position.y = aSensor.pose().position().y_m();
    lMsg.pose.position.z = aSensor.pose().position().z_m();

    lMsg.pose.orientation.x = aSensor.pose().rotation().x();
    lMsg.pose.orientation.y = aSensor.pose().rotation().y();
    lMsg.pose.orientation.z = aSensor.pose().rotation().z();
    lMsg.pose.orientation.w = aSensor.pose().rotation().w();

    lMsg.sensorID = aSensor.sensorID();

    mSensor_pub.publish(lMsg);
  }

} // namespace lidar_application