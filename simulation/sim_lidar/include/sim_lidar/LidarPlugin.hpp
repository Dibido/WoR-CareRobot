#ifndef LIDAR_PLUGIN_HPP
#define LIDAR_PLUGIN_HPP
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <sensor_interfaces/LidarData.h>
#include <sensor_msgs/LaserScan.h>

#include <ros/ros.h>

namespace gazebo
{
  class LidarPlugin : public RayPlugin
  {
      public:
    LidarPlugin() = default;

    virtual ~LidarPlugin() = default;

    /**
     * @brief Load the robot controller plugin, overrides the Load from
     * ModelPlugin
     * @param aParent: parent model
     * @param aSdf: the sdf (xml) in the robot model, within the <plugin>
     * element
     */
    void Load(sensors::SensorPtr aParent, sdf::ElementPtr aSdf);

      private:
    /**
     * @brief Callback function of gazebo sensor topic
     *        Publishes a LaserScan message on ros topic @var: topicName
     *        Publishes a LidarData message on ros topic /sensor/lidardata
     * @param aMsg: contains the information of the sensor
     */
    void OnScan(ConstLaserScanStampedPtr& aMsg);

    /**
     * @brief
     *
     */
    sensor_msgs::LaserScan convertToLaserScan(ConstLaserScanStampedPtr& aMsg);

    /**
     * @brief
     *
     * @param aMsg
     * @return sensor_interfaces::LidarData
     */
    sensor_interfaces::LidarData
        convertToLidarData(ConstLaserScanStampedPtr& aMsg);

    // Variables
    std::string mFrameName;
    std::string mTopicName;
    std::string mLidarDataTopic;
    sensors::RaySensorPtr mParentSensor;

    // Gazebo variables
    transport::NodePtr mGazeboNode;
    transport::SubscriberPtr mGazeboSub;

    // Ros variables
    ros::NodeHandlePtr mRosNode;
    ros::Publisher mRosPub;
    ros::Publisher mLidarDataPub;
  };
  GZ_REGISTER_SENSOR_PLUGIN(LidarPlugin)
} // namespace gazebo

#endif