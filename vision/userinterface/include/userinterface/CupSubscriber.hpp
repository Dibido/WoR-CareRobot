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
    /**
     * @brief Creates a CupSubscriber class. Do note that the CupSubscriber
     *        is not enabled by default and will not handle received messages
     * unless enabled.
     * @param aTopicName The topic name to listen on.
     */
    CupSubscriber(const std::string& aTopicName);
    virtual ~CupSubscriber();
    /**
     * @brief Needed to fullfill the ICupProvider interface. Does nothing
     * because this is a subscriber.
     */
    void passCup(const environment_controller::Cup& aCup);

    /**
     * @brief Sets whether the CupSubscriber should handle received messages.
     *
     * @param aValue enabled.
     */
    void setEnabled(bool aValue);

    /**
     * @brief Returns the arrival time of the cup at the gripper. If this is
     * zero, there is no arrival time.
     *
     * @return The cup arrival time.
     */
    ros::Time getArrivalTime();
    /**
     * @brief Returns the starting time of when the cup destination is received.
     * If this is zero, there is no arrival time.
     *
     * @return The cup arrival time.
     */
    ros::Time getStartingTime();
    /**
     * @brief Resets the cup start and arrival time to zero.
     */
    void resetArrivalTime();
    /**
     * @brief Resets all variables to zero.
     */
    void resetAll();

      private:
    /**
     * @brief The callback when a cup destination is received.
     */
    void cupCallback(const kinematica_msgs::CupConstPtr& aMsg);
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    /**
     * @brief To do ros::spin in a separate thread, a new function has to be
     * made.
     */
    static void threadRosSpin()
    {
      ros::spin();
    }
    /**
     * @brief Whether the CupSubscriber should handle received messages.
     */
    bool enabled = 0;
    /**
     * @brief Whether this is the first message that has been read.
     */
    bool mFirstMsgRead = 0;
    /**
     * The starting time of when the cup destination is received.
     */
    ros::Time mStartingTime;
    /**
     * The arrival time of the cup at the gripper.
     */
    ros::Time mCupArrivalTime;
  };

} // namespace userinterface

#endif /* CUPSUBSCRIBER_HPP */
