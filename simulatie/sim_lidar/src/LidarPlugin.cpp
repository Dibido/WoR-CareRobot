
#include <sensor_msgs/LaserScan.h>

#include <sim_lidar/LidarPlugin.hpp>

#define FRAME_NAME "frameName"
#define FRAME_NAME_DEFAULT "/laser"
#define TOPIC_NAME "topicName"
#define TOPIC_NAME_DEFAULT "/scan"

namespace gazebo
{
void LidarPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  RayPlugin::Load(_parent, _sdf);

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!parentSensor)
  {
    gzthrow("Lidar controller requires a Ray Sensor as its parent");
  }

  if (!_sdf->HasElement(FRAME_NAME))
  {
    ROS_INFO("Laser plugin missing <frameName>, defaults to %s", FRAME_NAME_DEFAULT);
    frameName = FRAME_NAME_DEFAULT;
  }
  else
  {
    frameName = _sdf->Get<std::string>(FRAME_NAME);
  }

  if (!_sdf->HasElement(TOPIC_NAME))
  {
    ROS_INFO("Laser plugin missing <topicName>, defaults to %s", TOPIC_NAME_DEFAULT);
    topicName = TOPIC_NAME_DEFAULT;
  }
  else
  {
    topicName = _sdf->Get<std::string>(TOPIC_NAME);
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
    // Advertise LaserScan msg on topic specified in plugin elekment <topicName>
    rosPub = rosNode->advertise<sensor_msgs::LaserScan>(topicName, 1);
  }
  else
  {
    ROS_WARN("Element topicName may not be empty!");
  }

  // Active the sensor
  parentSensor->SetActive(true);
}

// PRIVATE
void LidarPlugin::OnScan(ConstLaserScanStampedPtr& _msg)
{
  sensor_msgs::LaserScan laser_msg;

  laser_msg.header.stamp =
      ros::Time(static_cast<uint32_t>(_msg->time().sec()), static_cast<uint32_t>(_msg->time().nsec()));
  laser_msg.header.frame_id = frameName;
  laser_msg.angle_min = static_cast<float>(_msg->scan().angle_min());
  laser_msg.angle_max = static_cast<float>(_msg->scan().angle_max());
  laser_msg.time_increment = 0;
  laser_msg.scan_time = 0;
  laser_msg.range_min = static_cast<float>(_msg->scan().range_min());
  laser_msg.range_max = static_cast<float>(_msg->scan().range_max());
  laser_msg.ranges.resize(static_cast<unsigned long>(_msg->scan().ranges_size()));
  std::copy(_msg->scan().ranges().begin(), _msg->scan().ranges().end(), laser_msg.ranges.begin());
  laser_msg.angle_increment = static_cast<float>(2.0 * M_PI) / static_cast<float>(laser_msg.ranges.size());

  rosPub.publish(laser_msg);
}
}  // namespace gazebo
