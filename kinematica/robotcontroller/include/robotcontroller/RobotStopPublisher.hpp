#ifndef ROBOTSTOP_PUBLISHER_HPP
#define ROBOTSTOP_PUBLISHER_HPP

#include "robotcontroller_msgs/Stop.h"
#include "ros/ros.h"

namespace robotcontroller
{

  /**
   *
   * @author Brandon Geldof
   * @brief Publisher for stopping the robotarm
   *
   */
  class RobotStopPublisher
  {
      public:
    /**
     * @brief Construct a new Robot Stop Publisher object
     *
     * @param lN NodeHandler
     * @param lTopic Topic on which the message will be published
     * @param lQue_size Number of messages that will be qued
     */
    RobotStopPublisher(ros::NodeHandle& lN,
                       const std::string& lTopic,
                       const uint16_t lQue_size);

    /**
     * @brief Publishes a Stop msg to the robot_command topic
     *
     * @param stop
     */
    void publish(const bool lStop);

      private:
    ros::NodeHandle& mN;
    const std::string& cTopic;
    const uint16_t cQue_size;
    ros::Publisher mRobotControl_pub;
  };
} // namespace robotcontroller

#endif // ROBOTSTOP_PUBLISHER_HPP