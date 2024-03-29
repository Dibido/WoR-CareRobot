#ifndef DATAHANDLER_H
#define DATAHANDLER_H

#include "environment_controller/IObstacles.hpp"
#include "geometry_msgs/Point.h"
#include "lidar_application/LidarData.hpp"
#include <iostream>
#include <kinematica_msgs/Object.h>
#include <kinematica_msgs/Obstacles.h>
#include <ros/ros.h>
#include <sensor_interfaces/LidarData.h>

namespace lidar_application
{
  /**
   * @brief Datahandler takes care of input/output (lidar) data. It implements
   * the IObstacles interface, for more information see that description.
   */
  class DataHandler : public environment_controller::IObstacles
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
    DataHandler(const std::string& aReceiveTopic,
                const std::string& aPublishTopic);

    ~DataHandler() = default;

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
     * @pre: ros::init() must've been called
     * @post: Received data is stored in mLidarData, mNewDataAvailable will be
     * true
     * @param aLidarDataMessage - Size of its distances and measurement_angles
     * must be equal.
     */
    void dataReceiveCallback(
        const sensor_interfaces::LidarDataConstPtr& aLidarDataMessage);

    /**
     * @brief First translates data to the right type,
     * then publishes object data on topic using the parseObstacle() function.
     * @pre: -
     * @post: Data is published on rostopic
     * @param aData - X,Y of objects
     * @param aHeight - Z of the object
     */
    void publishData(const std::vector<std::pair<double, double>>& aData,
                     double aHeight_m);

    /**
     * @brief Publishes objects using mObjectPublisher
     * @param aObstacles - The obstacles
     */
    virtual void
        passObstacles(const environment_controller::Obstacles& aObstacles);

      private:
    ros::NodeHandle mNodeHandler;

    ros::Subscriber mLidarSubscriber;

    ros::Publisher mObjectPublisher;

    // Contains most recent lidar data
    LidarData mLidarData;

    bool mNewDataAvailable;
  };
} // namespace lidar_application

#endif /* DATAHANDLER_H */