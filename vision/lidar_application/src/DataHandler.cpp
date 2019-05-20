#include <DataHandler.h>

/**
 * @brief Default constructor
 */
DataHandler::DataHandler() : mNewDataAvailable(false)
{
  mLidarSubscriber = mNodeHandler.subscribe(
      "/sensor/lidar", 1000, &DataHandler::dataReceiveCallback, this);
}

/**
 * @brief Construct a new Data Handler:: Data Handler object
 * @param aReceiveTopic - Topic name on which data will be received
 * @param aPublishTopic - Topic name on which data will be published
 */
DataHandler::DataHandler(std::string& aReceiveTopic, std::string& aPublishTopic)
    : mNewDataAvailable(false)
{
  mLidarSubscriber = mNodeHandler.subscribe(
      aReceiveTopic, 1000, &DataHandler::dataReceiveCallback, this);

  mObjectPublisher =
      mNodeHandler.advertise<sensor_interfaces::LidarData>(aPublishTopic, 1000);
  // To-do publisher
}

DataHandler::~DataHandler()
{
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


  std::cout << "Angles: "
            << std::to_string(mLidarData.mAngles.size())
            << " distances: "
            << std::to_string(mLidarData.mDistances_m.size()) << std::endl;
}
