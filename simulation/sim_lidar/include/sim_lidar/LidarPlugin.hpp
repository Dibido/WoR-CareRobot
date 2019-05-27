#ifndef LIDAR_PLUGIN_HPP
#define LIDAR_PLUGIN_HPP

#include <sensor_interfaces/LidarData.h>
#include <sensor_msgs/LaserScan.h>

#include "lidar_application/ILidarData.hpp"
#include "lidar_application/LidarData.hpp"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <ros/ros.h>

#include <algorithm>
#include <iterator>

namespace gazebo
{
  class LidarPlugin : public RayPlugin, public lidar_application::ILidarData
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
     * @brief Implements the ILidarData interface.
     * @param aMsg - The message to parse.
     */
    virtual void parseLidarData(const lidar_application::LidarData& aMsg);

    /**
     * @brief Convert the gazebo::LaserScan const message to
     * sensor_msgs::LaserScan for processing
     * @param aMsg: The gazebo formatted message recieved from the simulation
     * @return: The converted sensor_msgs::LaserScan message
     */
    sensor_msgs::LaserScan convertToLaserScan(ConstLaserScanStampedPtr& aMsg);

    /**
     * @brief Convert the lidar_application::LidarData message to
     * sensor_msgs::LaserScan for sending
     * @param aMsg: The LidarData struct to be sent
     * @return: The converted sensor_msgs::LaserScan message
     */
    sensor_msgs::LaserScan
        convertToLaserScan(const lidar_application::LidarData aLidarData);

    /**
     * @brief Convert the LaserScan message
     * @param aMsg: The converted laserscan message
     * @return sensor_interfaces::LidarData: The LidarData message according to
     * the interface sensor_interfaces/LidarData.msg
     */
    sensor_interfaces::LidarData
        convertToLidarData(const sensor_msgs::LaserScan aMsg);

    /**
     * @brief Convert the LaserScan message
     * @param aMsg: The converted laserscan message
     * @return lidar_application::LidarData: The LidarData message according to
     * the interface ILidarData
     */
    lidar_application::LidarData
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