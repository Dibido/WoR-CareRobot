#include "lidar_application/DataHandler.hpp"

namespace lidar_application
{
  namespace datahandler_constants
  {
    const std::string cReceiveTopicName = "/sensor/lidar";
    const std::string cPublishTopicName = "/detectedObjects";
  } // namespace datahandler_constants

  /**
   * @brief Default constructor
   */
  DataHandler::DataHandler() : mNewDataAvailable(false)
  {
    mLidarSubscriber =
        mNodeHandler.subscribe(datahandler_constants::cReceiveTopicName, 1000,
                               &DataHandler::dataReceiveCallback, this);

    mObjectPublisher = mNodeHandler.advertise<kinematica_msgs::Obstacles>(
        datahandler_constants::cPublishTopicName, 1000);
  }

  /**
   * @brief Construct a new Data Handler:: Data Handler object
   * @param aReceiveTopic - Topic name on which data will be received
   * @param aPublishTopic - Topic name on which data will be published
   */
  DataHandler::DataHandler(const std::string& aReceiveTopic,
                           const std::string& aPublishTopic)
      : mNewDataAvailable(false)
  {
    mLidarSubscriber = mNodeHandler.subscribe(
        aReceiveTopic, 1000, &DataHandler::dataReceiveCallback, this);

    mObjectPublisher =
        mNodeHandler.advertise<kinematica_msgs::Obstacles>(aPublishTopic, 1000);
  }

  bool DataHandler::isNewDataAvailable() const
  {
    return mNewDataAvailable;
  }

  LidarData DataHandler::getLidarData()
  {
    mNewDataAvailable = false;
    return mLidarData;
  }

  void DataHandler::dataReceiveCallback(
      const sensor_interfaces::LidarDataConstPtr& aLidarDataMessage)
  {
    mNewDataAvailable = true;

    mLidarData.reset();

    for (size_t i = 0; i < aLidarDataMessage->distances.size(); ++i)
    {
      mLidarData.mAngles.push_back(
          static_cast<double>(aLidarDataMessage->measurement_angles.at(i)));
      mLidarData.mDistances_m.push_back(
          static_cast<double>(aLidarDataMessage->distances.at(i)));
    }
  }

  void DataHandler::publishData(
      const std::vector<std::pair<double, double>>& aData,
      double aHeight_m)
  {
    environment_controller::Obstacles lObstacleList;

    geometry_msgs::Point lPoint;

    for (size_t i = 0; i < aData.size(); ++i)
    {
      environment_controller::Position lPosition(aData.at(i).first, aData.at(i).second, aHeight_m);
      
      environment_controller::Object lObject(lPosition, 0.0, 0.0, 0.0, 0.0,
                                           0.0, ros::Time(), 0);

      lObstacleList.push_back(lObject);
    }

    parseObstacles(lObstacleList);
  }

  void DataHandler::parseObstacles(
      const environment_controller::Obstacles& aObstacles)
  {
    kinematica_msgs::Object lObject;
    kinematica_msgs::Obstacles lObstacles;

    geometry_msgs::Point lPoint;

    for(size_t i = 0; i < aObstacles.size(); ++i)
    {
      lPoint.x = aObstacles.at(i).position().x_m();
      lPoint.y = aObstacles.at(i).position().y_m();
      lPoint.z = aObstacles.at(i).position().z_m();

      lObject.position_m = lPoint;
      lObstacles.obstacles.push_back(lObject);
    }

    mObjectPublisher.publish(lObstacles);
    ros::spinOnce();
  }
} // namespace lidar_application