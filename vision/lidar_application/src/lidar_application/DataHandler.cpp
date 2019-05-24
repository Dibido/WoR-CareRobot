#include "DataHandler.hpp"

namespace lidar_application
{
namespace datahandler_constants
{
  const std::string cReceiveTopicName = "/sensor/lidar";
  const std::string cPublishTopicName = "/detectedObjects";
}

/**
 * @brief Default constructor
 */
DataHandler::DataHandler() : mNewDataAvailable(false)
{
  mLidarSubscriber = mNodeHandler.subscribe(
      datahandler_constants::cReceiveTopicName, 1000, &DataHandler::dataReceiveCallback, this);

  mObjectPublisher = mNodeHandler.advertise<kinematica_msgs::Obstacles>(datahandler_constants::cPublishTopicName, 1000);
}

/**
 * @brief Construct a new Data Handler:: Data Handler object
 * @param aReceiveTopic - Topic name on which data will be received
 * @param aPublishTopic - Topic name on which data will be published
 */
DataHandler::DataHandler(const std::string& aReceiveTopic, const std::string& aPublishTopic)
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

void DataHandler::publishData(const std::vector<std::pair<double, double>>& aData, double aHeight_m) const
{
    kinematica_msgs::Object lObject;
    kinematica_msgs::Obstacles lObstacleList;

    geometry_msgs::Point lPoint;

    for(size_t i = 0; i < aData.size(); ++i)
    {
      lPoint.x = aData.at(i).first;
      lPoint.y = aData.at(i).second;
      lPoint.z = aHeight_m;

      lObject.position_m = lPoint;
      lObstacleList.obstacles.push_back(lObject);
    }

    mObjectPublisher.publish(lObstacleList);
    ros::spinOnce();
}
}