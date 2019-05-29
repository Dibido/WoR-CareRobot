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
      Position lPos(aMsg->mX_m, aMsg->mY_m, aMsg->mZ_m);
      Object lObj(lPos, aMsg->aHeight, aMsg->aWidth, aMsg->aDepth,
                  aMsg->aDirection, aMsg->aSpeed, aMsg->aMeasurementTime,
                  aMsg->aSensorId);
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