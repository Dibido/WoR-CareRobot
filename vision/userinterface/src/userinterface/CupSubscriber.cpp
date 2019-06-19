#include "userinterface/CupSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

namespace userinterface
{
  CupSubscriber::CupSubscriber()
      : mSubscriber(
            mHandle.subscribe("cup",
                              10,
                              &userinterface::CupSubscriber::cupCallback,
                              this))
  {
    std::cout << "TEST" << std::endl;

    std::thread lSpinThread(threadRosSpin);
    lSpinThread.detach();
  }

  CupSubscriber::~CupSubscriber()
  {
  }

  CupSubscriber::CupSubscriber(const std::string& aTopicName)
      : mSubscriber(
            mHandle.subscribe(aTopicName,
                              10,
                              &userinterface::CupSubscriber::cupCallback,
                              this))

  {
  }

  void CupSubscriber::cupCallback(const kinematica_msgs::CupConstPtr& aMsg)
  {
    try
    {
      //      Position lPos(aMsg->mX_m, aMsg->mY_m, aMsg->mZ_m);
      //      Object lObj(lPos, aMsg->aHeight, aMsg->aWidth, aMsg->aDepth,
      //                  aMsg->aDirection, aMsg->aSpeed,
      //                  aMsg->aMeasurementTime, aMsg->aSensorId);
      //      Cup lCup(lObj, aMsg->timeOfArrival);
      if (enabled)
      {
        std::cout << std::to_string(aMsg->timeOfArrival.toSec()) << std::endl;
        mStartingTime = ros::Time::now().toSec();
        if (mCupArrivalTime == 0)
        {
          mFirstMsgRead = true;
        }
        mCupArrivalTime = aMsg->timeOfArrival.toSec();
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("%s", e.what());
    }
  }

  void CupSubscriber::passCup(const environment_controller::Cup& aCup)
  {
  }

  void CupSubscriber::setEnabled(bool aValue)
  {
    enabled = aValue;
  }

  uint16_t CupSubscriber::getArrivalTime()
  {
    return mCupArrivalTime;
  }

  double CupSubscriber::getStartingTime()
  {
    return mStartingTime;
  }

} // namespace userinterface
