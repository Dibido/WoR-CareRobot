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
     * @brief Convert the gazebo::LaserScan const message to
     * sensor_msgs::LaserScan for processing
     * @param aMsg: The gazebo formatted message recieved from the simulation
     * @return: The converted sensor_msgs::LaserScan message
     */
    sensor_msgs::LaserScan
        convertToLaserScan(ConstLaserScanStampedPtr& aMsg) const;

    /**
     * @brief Convert the LaserScan message to
     * @param aMsg: The converted laserscan message
     * @return sensor_interfaces::LidarData: The LidarData message according to
     * the interface sensor_interfaces/LidarData.msg
     */
    sensor_interfaces::LidarData
        convertToLidarData(const sensor_msgs::LaserScan aMsg) const;

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