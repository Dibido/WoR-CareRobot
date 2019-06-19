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
    void resetArrivalTime();
    uint16_t getArrivalTime();
    double getStartingTime();

      private:
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    static void threadRosSpin()
    {
      ros::spin();
    }
    bool enabled = 0;
    bool mFirstMsgRead = 0;
    double mStartingTime = 0.0;
    double mCupArrivalTime = 0.0;
  };

} // namespace userinterface

#endif /* CUPSUBSCRIBER_HPP */
