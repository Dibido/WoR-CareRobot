#include <sim_lidar/LidarPlugin.hpp>

namespace LidarConfiguration
{
  const std::string cFrameName = "frameName";
  const std::string cFrameNameDefault = "/laser";
  const std::string cTopicName = "topicName";
  const std::string cTopicNameDefault = "/scan";
  // Set lidardata topic as defined in LidarData.msg
  const std::string cLidarDataTopicName = "lidardatatopicName";
  const std::string cLidarDataTopic = "/sensor/lidardata";

} // namespace LidarConfiguration

namespace gazebo
{
  void LidarPlugin::Load(sensors::SensorPtr aParent, sdf::ElementPtr aSdf)
  {
    RayPlugin::Load(aParent, aSdf);

    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM(
          "A ROS node for Gazebo has not been initialized, unable to load "
          "plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
             "the gazebo_ros package)");
      return;
    }

    mParentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(aParent);

    if (!mParentSensor)
    {
      gzthrow("Lidar controller requires a Ray Sensor as its parent");
    }

    if (!aSdf->HasElement(LidarConfiguration::cFrameName))
    {
      ROS_INFO("Laser plugin missing <frameName>, defaults to %s",
               LidarConfiguration::cFrameNameDefault);
      mFrameName = LidarConfiguration::cFrameNameDefault;
    }
    else
    {
      mFrameName = aSdf->Get<std::string>(LidarConfiguration::cFrameName);
    }

    if (!aSdf->HasElement(LidarConfiguration::cTopicName))
    {
      ROS_INFO("Laser plugin missing <topicName>, defaults to %s",
               LidarConfiguration::cTopicNameDefault);
      mTopicName = LidarConfiguration::cTopicNameDefault;
    }
    else
    {
      mTopicName = aSdf->Get<std::string>(LidarConfiguration::cTopicName);
    }

    if (!aSdf->HasElement(LidarConfiguration::cLidarDataTopicName))
    {
      ROS_INFO("Laser plugin missing <lidardatatopicName>, defaults to %s",
               LidarConfiguration::cLidarDataTopic);
      mLidarDataTopic = LidarConfiguration::cLidarDataTopic;
    }
    else
    {
      mLidarDataTopic =
          aSdf->Get<std::string>(LidarConfiguration::cLidarDataTopicName);
    }

    // Start gazebo node
    mGazeboNode = transport::NodePtr(new transport::Node());
    mGazeboNode->Init();

    // Subscribe to gazebo sensor topic
    mGazeboSub = mGazeboNode->Subscribe(mParentSensor->Topic(),
                                        &LidarPlugin::OnScan, this);

    // Create a ros NodeHandle
    mRosNode = std::make_unique<ros::NodeHandle>();

    if (!mTopicName.empty())
    {
      // Advertise LaserScan msg on topic specified in plugin element
      // <topicName>
      mRosPub = mRosNode->advertise<sensor_msgs::LaserScan>(mTopicName, 1);
    }
    else
    {
      ROS_WARN("Element topicName may not be empty!");
    }
    if (!mLidarDataTopic.empty())
    {
      // Advertise LaserScan msg on topic specified in plugin element
      // <lidardatatopicName>
      mLidarDataPub =
          mRosNode->advertise<sensor_interfaces::LidarData>(mLidarDataTopic, 1);
    }
    else
    {
      ROS_WARN("Element lidardatatopicName may not be empty!");
    }

    // Active the sensor
    mParentSensor->SetActive(true);
  }

  // PRIVATE
  void LidarPlugin::OnScan(ConstLaserScanStampedPtr& aMsg)
  {
    // Convert to LidarData
    lidar_application::LidarData lLidarData = convertToLidarData(aMsg);
    // Handle the LidarData
    parseLidarData(lLidarData);
  }

  void LidarPlugin::parseLidarData(const lidar_application::LidarData& aMsg)
  {
    // Convert to LaserScan message
    sensor_msgs::LaserScan lLaserMessage = convertToLaserScan(aMsg);
    // Publish message
    mRosPub.publish(lLaserMessage);

    // Convert to lidarMessage
    sensor_interfaces::LidarData lLidarDataMessage =
        convertToLidarData(lLaserMessage);
    // Check sizes
    assert(lLidarDataMessage.distances.size() ==
           lLidarDataMessage.measurement_angles.size());
    // Publish message
    mLidarDataPub.publish(lLidarDataMessage);
  }

  sensor_msgs::LaserScan
      LidarPlugin::convertToLaserScan(ConstLaserScanStampedPtr& aMsg) const
  {
    sensor_msgs::LaserScan lLaserMessage;
    float lAngleMin = static_cast<float>(aMsg->scan().angle_min());
    float lAngleMax = static_cast<float>(aMsg->scan().angle_max());
    lLaserMessage.header.stamp =
        ros::Time(static_cast<uint32_t>(aMsg->time().sec()),
                  static_cast<uint32_t>(aMsg->time().nsec()));
    lLaserMessage.header.frame_id = mFrameName;
    lLaserMessage.angle_min = lAngleMin;
    lLaserMessage.angle_max = lAngleMax;
    lLaserMessage.time_increment = 0;
    lLaserMessage.scan_time = 0;
    lLaserMessage.range_min = static_cast<float>(aMsg->scan().range_min());
    lLaserMessage.range_max = static_cast<float>(aMsg->scan().range_max());
    lLaserMessage.ranges.resize(
        static_cast<unsigned long>(aMsg->scan().ranges().size()));

    std::copy(aMsg->scan().ranges().begin(), aMsg->scan().ranges().end(),
              lLaserMessage.ranges.begin());

    lLaserMessage.angle_increment =
        static_cast<float>(2.0 * M_PI) /
        static_cast<float>(lLaserMessage.ranges.size());

    return lLaserMessage;
  }

  sensor_msgs::LaserScan LidarPlugin::convertToLaserScan(
      const lidar_application::LidarData aLidarData) const
  {
    sensor_msgs::LaserScan lLaserMessage;
    lLaserMessage.range_min = static_cast<float>(-M_PI);
    lLaserMessage.range_max = static_cast<float>(M_PI);
    lLaserMessage.ranges.resize(
        static_cast<unsigned long>(aLidarData.mDistances_m.size()));

    // Reverse the measured distances so the scan is taken from left to right.
    std::reverse_copy(aLidarData.mDistances_m.begin(),
                      aLidarData.mDistances_m.end(),
                      lLaserMessage.ranges.begin());

    lLaserMessage.angle_increment =
        static_cast<float>(2.0 * M_PI) /
        static_cast<float>(lLaserMessage.ranges.size());
    return lLaserMessage;
  }

  lidar_application::LidarData
      LidarPlugin::convertToLidarData(ConstLaserScanStampedPtr& aMsg) const
  {
    lidar_application::LidarData lLidarData;
    auto lCounter = 0;
    float lAngleMax = static_cast<float>(M_PI);

    //  calculate::Calculatedata lCalc;
    //c lCalc.processData();

    generate::GenerateNoise lNoise;

    lNoise.GenerateNoiseSample(0.476773, 0.125522);
    // Fill the angle array with the correct range. Add 1 PI so we get a range
    // from 0..2 * Pi.

    for (double lCurrentAngle = lNoise.mNoise[lCounter];
         lCurrentAngle < (lAngleMax + static_cast<float>(M_PI));
         lCurrentAngle += lNoise.mNoise[lCounter])
    {
      lLidarData.mAngles.push_back(lCurrentAngle);
      ++lCounter;
    }
    for (int i = 0; i < aMsg->scan().ranges().size(); i++)
    {
      lLidarData.mDistances_m.push_back(aMsg->scan().ranges().Get(i));
    }
    return lLidarData;
  }

  sensor_interfaces::LidarData
      LidarPlugin::convertToLidarData(const sensor_msgs::LaserScan aMsg) const
  {
    sensor_interfaces::LidarData lLidarDataMessage;
    lLidarDataMessage.header.stamp = ros::Time::now();
    lLidarDataMessage.header.frame_id = mFrameName;
    // Add distances
    lLidarDataMessage.distances.resize(
        static_cast<unsigned long>(aMsg.ranges.size()));
    std::copy(aMsg.ranges.begin(), aMsg.ranges.end(),
              lLidarDataMessage.distances.begin());
    // Add angles
    lLidarDataMessage.measurement_angles.clear();
    // Fill the angle array with the correct range. Add 1 PI so we get a range
    // from 0..2 * Pi.
    const float lAngleOffset =
        static_cast<float>(2.0 * M_PI) / static_cast<float>(aMsg.ranges.size());
    for (float lCurrentAngle = lAngleOffset;
         lCurrentAngle < (static_cast<float>(M_PI * 2));
         lCurrentAngle += lAngleOffset)
    {
      lLidarDataMessage.measurement_angles.push_back(lCurrentAngle);
    }
    return lLidarDataMessage;
  }

} // namespace gazebo
