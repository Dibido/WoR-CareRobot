/**
 * @file ObstacleSubsciber.hpp
 * @author Gianni Monteban
 * @brief
 * @version 0.1
 * @date 2019-05-20
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef OBSTACLE_SUBSCRIBER_HPP
#define OBSTACLE_SUBSCRIBER_HPP

#include "kinematica_msgs/Obstacles.h"
#include "ros/ros.h"
#include <string>

namespace environment_controller
{
  const uint8_t QUEUE_SIZE = 100;
  class ObstacleSubsciber
  {

      public:
    ObstacleSubsciber(std::string& aSubName);

    virtual ~ObstacleSubsciber() = default;

    void obstaclesCallback(const kinematica_msgs::ObstaclesConstPtr& aMsg);

      private:
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    ros::NodeHandle mHandlePub;
  };
} // namespace obstacle

#endif // OBSTACLE_SUBSCRIBER_HPP