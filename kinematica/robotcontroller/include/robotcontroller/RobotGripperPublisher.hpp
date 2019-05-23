#ifndef ROBOTCONTROL_PUBLISHER_HPP
#define ROBOTCONTROL_PUBLISHER_HPP

#include "robotcontroller/IGripperControl.hpp"
#include "robotcontroller_msgs/Gripper.h"
#include "ros/ros.h"

#include <iostream>

namespace robotcontroller
{

  /**
   *
   * @author Brandon Geldof
   * @brief Publisher for controlling the robotarm
   *
   */
  class RobotGripperPublisher : public IGripperControl
  {
      public:
    /**
     * @brief Construct a new Robot Control Publisher object
     *
     * @param lN NodeHandler
     * @param lTopic Topic on which the message will be published
     * @param lQueue_size Number of messages that will be queued
     */
    RobotGripperPublisher(ros::NodeHandle& lN,
                          const std::string& lTopic,
                          const uint16_t lQueue_size);

    ~RobotGripperPublisher() = default;

    /**
     * @brief Publishes a Control msg to the robot_command topic
     *
     * @param lSf Speedfactor for the robotarm movement speed
     * @param lConfiguration Configuration of the robotarm
     */
    virtual void moveGripper(const robotcontroller::GripperData& aGripperData);

      private:
    ros::NodeHandle& mN;
    const std::string& cTopic;
    const uint16_t cQueue_size;
    ros::Publisher mRobotGripper_pub;
  };
} // namespace robotcontroller

#endif // ROBOTCONTROL_PUBLISHER_HPP