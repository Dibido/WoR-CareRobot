#include "userinterface/CupSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

namespace userinterface
{
  CupSubscriber::CupSubscriber()
      : mSubscriber(
            mHandle.subscribe("/location/cup",
                              1000,
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
                              1000,
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
        mStartingTime = ros::Time::now();
	mCupArrivalTime = aMsg->timeOfArrival;
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
    // mCupArrivalTime = 0.0;
    // mStartingTime = 0.0;
  }

  void CupSubscriber::resetAll()
  {
    resetArrivalTime();
    bool enabled = 0;
    bool mFirstMsgRead = 0;
  }

  ros::Time CupSubscriber::getArrivalTime()
  {
    return mCupArrivalTime;
  }

  ros::Time CupSubscriber::getStartingTime()
  {
    return mStartingTime;
  }

} // namespace userinterface
