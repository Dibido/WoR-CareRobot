/**
 * @file ObstaclesSubscriber.hpp
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

#include "IObstacles.hpp"
#include "kinematica_msgs/Obstacles.h"
#include "ros/ros.h"
#include <string>

namespace environment_controller
{
  const uint8_t cQueueSize = 100;

  /**
   * @brief implements the interface for obstacles, this class receives the ros
   * messages for obstacles
   *
   */
  class ObstaclesSubscriber : public IObstacles
  {

      public:
    /**
     * @brief Construct a new Obstacles Subscriber object
     *
     * @param aSubName
     *
     * @author Gianni Monteban
     */
    ObstaclesSubscriber(const std::string& aSubName);

    /**
     * @brief Destroy the Obstacles Subscriber object
     *
     * @author Gianni Monteban
     */
    virtual ~ObstaclesSubscriber() = default;

    /**
     * @brief receives the message from
     *
     * @param aMsg the message that is received on the obstacle topic
     *
     * @author Gianni Monteban
     */
    void obstaclesCallback(const kinematica_msgs::ObstaclesConstPtr& aMsg);

    /**
     * @see IObstacles.hpp
     *
     * @author Gianni Monteban
     */
    void parseObstacles(const Obstacles& aObstacles);

      private:
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    ros::NodeHandle mHandlePub;
  };
} // namespace environment_controller

#endif // OBSTACLE_SUBSCRIBER_HPP