#ifndef CUPSUBSCRIBER_HPP
#define CUPSUBSCRIBER_HPP

#include "environment_controller/Cup.hpp"
#include "environment_controller/ICupProvider.hpp"
#include "kinematica_msgs/Cup.h"
#include "ros/ros.h"
#include <thread>

namespace userinterface
{
  class CupSubscriber : public environment_controller::ICupProvider
  {
      public:
    CupSubscriber();
    CupSubscriber(const std::string& aTopicName);
    virtual ~CupSubscriber();
    void cupCallback(const kinematica_msgs::CupConstPtr& aMsg);
    void passCup(const environment_controller::Cup& aCup);

    void setEnabled(bool aValue);
    ros::Time getArrivalTime();
    ros::Time getStartingTime();
    void resetArrivalTime();
    void resetAll();

      private:
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    static void threadRosSpin()
    {
      ros::spin();
    }
    bool enabled = 0;
    bool mFirstMsgRead = 0;
    ros::Time mStartingTime;
    ros::Time mCupArrivalTime;
  };

} // namespace userinterface

#endif /* CUPSUBSCRIBER_HPP */
