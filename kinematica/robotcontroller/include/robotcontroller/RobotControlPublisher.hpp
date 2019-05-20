#ifndef ROBOTCONTROL_PUBLISHER_HPP
#define ROBOTCONTROL_PUBLISHER_HPP

#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include "robotcontroller_msgs/Control.h"
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
  class RobotControlPublisher
  {
      public:
    /**
     * @brief Construct a new Robot Control Publisher object
     *
     * @param lN NodeHandler
     * @param lTopic Topic on which the message will be published
     * @param lQue_size Number of messages that will be qued
     */
    explicit RobotControlPublisher(ros::NodeHandle& lN,
                                   const std::string& lTopic,
                                   const uint16_t lQue_size);

    /**
     * @brief Publishes a Control msg to the robot_command topic
     *
     * @param lSf Speedfactor for the robotarm movement speed
     * @param lConfiguration Configuration of the robotarm
     */
    void publish(const double lSf, const std::vector<double>& lConfiguration);

      private:
    ros::NodeHandle& mN;
    const std::string& cTopic;
    const uint16_t cQue_size;
    ros::Publisher mRobotControl_pub;
  };
} // namespace robotcontroller

#endif // ROBOTCONTROL_PUBLISHER_HPP