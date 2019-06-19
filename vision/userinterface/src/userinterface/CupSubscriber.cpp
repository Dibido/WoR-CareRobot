#include "userinterface/CupSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

namespace userinterface
{
  CupSubscriber::CupSubscriber()
      : mSubscriber(
            mHandle.subscribe("/location/cup",
                              10,
                              &userinterface::CupSubscriber::cupCallback,
                              this))
  {
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
      if (enabled)
      {
        mStartingTime = ros::Time::now().toSec();
        if (mCupArrivalTime == 0.0)
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

  void CupSubscriber::resetArrivalTime()
  {
    mCupArrivalTime = 0.0;
    mStartingTime = 0.0;
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
