#ifndef DATAHANDLER_H
#define DATAHANDLER_H

#include "../include/LidarData.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_interfaces/LidarData.h>

#include <iostream>

//TODO: Include messages here

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
   * @precondition: -
   * @postcondition: mNewDataAvailable will be false
   * @return LidarData 
   */
  LidarData getLidarData();
  
  void dataReceiveCallback(const sensor_interfaces::LidarDataConstPtr& aLidarDataMessage);

  private:
  ros::NodeHandle mNodeHandler;

  ros::Subscriber mLidarSubscriber;

  ros::Publisher mObjectPublisher;

  // Contains most recent lidar data
  LidarData mLidarData;

  bool mNewDataAvailable;
};

#endif /* DATAHANDLER_H */