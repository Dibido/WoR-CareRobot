#include "location_component/RosServiceCup.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

namespace location_component
{

  RosServiceCup::RosServiceCup(ros::NodeHandle& aNodeHandle)
      : mNodeHandle(aNodeHandle)
  {
    mCupPublisher = aNodeHandle.advertise<kinematica_msgs::Cup>(
        environment_controller::cCupTopicName, 1000);
  }

  void RosServiceCup::foundCup(const environment_controller::Cup& aCup)
  {
    kinematica_msgs::Cup object;

    object.aDepth = aCup.object().depth_m();
    object.aDirection = aCup.object().direction_rad();
    object.aHeight = aCup.object().height_m();
    object.aMeasurementTime = aCup.object().measurementTime();
    object.aSensorId = aCup.object().sensorId();
    object.aSpeed = aCup.object().speed_ms();
    object.aWidth = aCup.object().width_m();

    object.mX_m = aCup.object().position().x_m();
    object.mY_m = aCup.object().position().y_m();
    object.mZ_m = aCup.object().position().z_m();

    object.timeOfArrival = aCup.timeOfArrival();

    mCupPublisher.publish(object);
    ros::spinOnce();
  }
} // namespace location_component
