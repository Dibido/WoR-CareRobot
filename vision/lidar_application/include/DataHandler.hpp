#ifndef DATAHANDLER_H
#define DATAHANDLER_H

#include "../include/LidarData.hpp"

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_interfaces/LidarData.h>
#include <kinematica_msgs/Object.h>
#include <kinematica_msgs/Obstacles.h>

#include "geometry_msgs/Point.h"

#include <iostream>


class DataHandler
{
  public:
  /**
   * @brief Default constructor
   */
  DataHandler();

  /**
   * @brief Construct a new Data Handler:: Data Handler object
   * @param aReceiveTopic - Topic name on which data will be received
   * @param aPublishTopic - Topic name on which data will be published
   */
  DataHandler(std::string& aReceiveTopic, std::string& aPublishTopic);

  ~DataHandler();

  bool isNewDataAvailable() const;

  /**
   * @brief Returns mLidarData, the most recently received lidar data
   * @pre: -
   * @post: mNewDataAvailable will be false
   * @return LidarData 
   */
  LidarData getLidarData();
  
  /**
   * @brief Handles incoming messages on defined topic (see constants in cpp)
   * @pre: ros::init() must've ben called
   * @post: Received data is stored in mLidarData, mNewDataAvailable will be true
   * @param aLidarDataMessage 
   */
  void dataReceiveCallback(const sensor_interfaces::LidarDataConstPtr& aLidarDataMessage);

  /**
   * @brief Publishes object data on topic
   * @pre: -
   * @post: Data is published on rostopic
   * @param aData - X,Y of objects
   * @param aHeight - Z of the object
   */
  void publishData(std::vector<std::pair<double, double>>& aData, double aHeight_m) const;

  private:
  ros::NodeHandle mNodeHandler;

  ros::Subscriber mLidarSubscriber;

  ros::Publisher mObjectPublisher;

  // Contains most recent lidar data
  LidarData mLidarData;

  bool mNewDataAvailable;
};

#endif /* DATAHANDLER_H */