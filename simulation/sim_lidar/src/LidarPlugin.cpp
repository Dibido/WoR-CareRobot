#include <sensor_msgs/LaserScan.h>
#include <sim_lidar/LidarPlugin.hpp>
#include <sensor_interfaces/LidarData.h>

const std::string cFrameName = "frameName";
const std::string cFrameNameDefault = "/laser";
const std::string cTopicName = "topicName";
const std::string cTopicNameDefault = "/scan";
// Set lidardata topic as defined in LidarData.msg
const std::string cLidarDataTopicName = "lidardatatopicName";
const std::string cLidarDataTopic = "/sensor/lidardata";

namespace gazebo
{
void LidarPlugin::Load(sensors::SensorPtr aParent, sdf::ElementPtr aSdf)
{
  RayPlugin::Load(aParent, aSdf);

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(aParent);

  if (!parentSensor)
  {
    gzthrow("Lidar controller requires a Ray Sensor as its parent");
  }

  if (!aSdf->HasElement(cFrameName))
  {
    ROS_INFO("Laser plugin missing <frameName>, defaults to %s", cFrameNameDefault);
    frameName = cFrameNameDefault;
  }
  else
  {
    frameName = aSdf->Get<std::string>(cFrameName);
  }

  if (!aSdf->HasElement(cTopicName))
  {
    ROS_INFO("Laser plugin missing <topicName>, defaults to %s", cTopicNameDefault);
    topicName = cTopicNameDefault;
  }
  else
  {
    topicName = aSdf->Get<std::string>(cTopicName);
  }

  if (!aSdf->HasElement(cLidarDataTopicName))
  {
    ROS_INFO("Laser plugin missing <lidardatatopicName>, defaults to %s", cLidarDataTopic);
    mLidarDataTopic = cLidarDataTopic;
  }
  else
  {
    mLidarDataTopic = aSdf->Get<std::string>(cLidarDataTopicName);
  }

  // Start gazebo node
  gazeboNode = transport::NodePtr(new transport::Node());
  gazeboNode->Init();

  // Subscribe to gazebo sensor topic
  gazeboSub = gazeboNode->Subscribe(parentSensor->Topic(), &LidarPlugin::OnScan, this);

  // Create a ros NodeHandle
  rosNode = std::make_unique<ros::NodeHandle>();

  if (!topicName.empty())
  {
    // Advertise LaserScan msg on topic specified in plugin element <topicName>
    rosPub = rosNode->advertise<sensor_msgs::LaserScan>(topicName, 1);
    
  }
  else
  {
    ROS_WARN("Element topicName may not be empty!");
  }
  if (!mLidarDataTopic.empty())
  {
    // Advertise LaserScan msg on topic specified in plugin element <lidardatatopicName>
    mLidarDataPub = rosNode->advertise<sensor_interfaces::LidarData>(mLidarDataTopic, 1);
  }
  else
  {
    ROS_WARN("Element lidardatatopicName may not be empty!");
  }

  // Active the sensor
  parentSensor->SetActive(true);
}

// PRIVATE
void LidarPlugin::OnScan(ConstLaserScanStampedPtr& aMsg)
{
  //Publish the LaserScan message
  sensor_msgs::LaserScan lLaserMessage;
  float lAngleMin = static_cast<float>(aMsg->scan().angle_min());
  float lAngleMax = static_cast<float>(aMsg->scan().angle_max());

  lLaserMessage.header.stamp =
      ros::Time(static_cast<uint32_t>(aMsg->time().sec()), static_cast<uint32_t>(aMsg->time().nsec()));
  lLaserMessage.header.frame_id = frameName;
  lLaserMessage.angle_min = lAngleMin;
  lLaserMessage.angle_max = lAngleMax;
  lLaserMessage.time_increment = 0;
  lLaserMessage.scan_time = 0;
  lLaserMessage.range_min = static_cast<float>(aMsg->scan().range_min());
  lLaserMessage.range_max = static_cast<float>(aMsg->scan().range_max());
  lLaserMessage.ranges.resize(static_cast<unsigned long>(aMsg->scan().ranges_size()));
  std::copy(aMsg->scan().ranges().begin(), aMsg->scan().ranges().end(), lLaserMessage.ranges.begin());
  lLaserMessage.angle_increment = static_cast<float>(2.0 * M_PI) / static_cast<float>(lLaserMessage.ranges.size());
  rosPub.publish(lLaserMessage);

  // Publish the LidarData message
  sensor_interfaces::LidarData lLidarDataMessage;
  lLidarDataMessage.header.stamp =
      ros::Time(static_cast<uint32_t>(aMsg->time().sec()), static_cast<uint32_t>(aMsg->time().nsec()));
  lLidarDataMessage.header.frame_id = frameName;
  // Add distances
  lLidarDataMessage.distances.resize(static_cast<unsigned long>(aMsg->scan().ranges_size()));
  std::copy(aMsg->scan().ranges().begin(), aMsg->scan().ranges().end(), lLidarDataMessage.distances.begin());
  // Add angles
  lLidarDataMessage.measurement_angles.clear();
  // Fill the angle array with the correct range. Add 1 PI so we get a range from 0 - TAU.
  float lAngleOffset = static_cast<float>(2.0 * M_PI) / static_cast<float>(lLaserMessage.ranges.size());
  for(float lCurrentAngle = lAngleOffset; lCurrentAngle < (lAngleMax + static_cast<float>(M_PI)); lCurrentAngle += lAngleOffset)
  {
    lLidarDataMessage.measurement_angles.push_back(lCurrentAngle);
  }
  //Assert to check that the ranges and their angles are matched
  assert(lLidarDataMessage.measurement_angles.size() == lLidarDataMessage.distances.size());
  mLidarDataPub.publish(lLidarDataMessage);
}
}  // namespace gazebo
