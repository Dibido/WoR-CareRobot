#include "lidar_application/DataHandler.hpp"

namespace lidar_application
{
  namespace datahandler_constants
  {
    const std::string cReceiveTopicName = "/sensor/lidar";
    const std::string cPublishTopicName = "/detected_objects";
  } // namespace datahandler_constants

  DataHandler::DataHandler() : mNewDataAvailable(false)
  {
    mLidarSubscriber =
        mNodeHandler.subscribe(datahandler_constants::cReceiveTopicName, 1000,
                               &DataHandler::dataReceiveCallback, this);

    mObjectPublisher = mNodeHandler.advertise<kinematica_msgs::Obstacles>(
        datahandler_constants::cPublishTopicName, 1000);
  }

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
    if (aLidarDataMessage->distances.size() !=
        aLidarDataMessage->measurement_angles.size())
    {
      throw std::logic_error(
          "dataReceiveCallback: distances.size and measurement_angles.size of "
          "aLidarDataMessage must be equal");
    }

    mNewDataAvailable = true;

    mLidarData.reset();

    for (size_t i = 0; i < aLidarDataMessage->distances.size(); ++i)
    {
      mLidarData.mMeasurements.insert(
          std::pair<double, double>(aLidarDataMessage->measurement_angles.at(i),
                                    aLidarDataMessage->distances.at(i)));
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
      environment_controller::Position lPosition(aData.at(i).first,
                                                 aData.at(i).second, aHeight_m);

      environment_controller::Object lObject(lPosition, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             ros::Time(), 0);

      lObstacleList.push_back(lObject);
    }

    passObstacles(lObstacleList);
  }

  void DataHandler::passObstacles(
      const environment_controller::Obstacles& aObstacles)
  {
    kinematica_msgs::Object lObject;
    kinematica_msgs::Obstacles lObstacles;

    geometry_msgs::Point lPoint;

    for (size_t i = 0; i < aObstacles.size(); ++i)
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