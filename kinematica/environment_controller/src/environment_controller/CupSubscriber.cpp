#include "environment_controller/CupSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

namespace environment_controller
{
  CupSubscriber::CupSubscriber(
      const std::string& aTopicName,
      const std::shared_ptr<EnvironmentController>& aController)
      : mSubscriber(mHandle.subscribe(
            aTopicName,
            cQueue_size,
            &environment_controller::CupSubscriber::cupCallback,
            this)),
        mEnvironmentController(aController)

  {
  }

  void CupSubscriber::cupCallback(const kinematica_msgs::CupConstPtr& aMsg)
  {
    try
    {
      Position lPos(aMsg->cup.position_m.x, aMsg->cup.position_m.y,
                    aMsg->cup.position_m.z);
      Object lObj(lPos, aMsg->cup.height_m, aMsg->cup.width_m,
                  aMsg->cup.depth_m, aMsg->cup.direction_rad,
                  aMsg->cup.speed_ms, aMsg->cup.measurementTime,
                  aMsg->cup.sensorID);
      Cup lCup(lObj, aMsg->timeOfArrival);
      foundCup(lCup);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("%s", e.what());
    }
  }

  void CupSubscriber::foundCup(const Cup& aCup)
  {
    mEnvironmentController->provideCup(aCup);
  }

} // namespace environment_controller