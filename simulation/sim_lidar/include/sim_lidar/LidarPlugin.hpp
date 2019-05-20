
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <ros/ros.h>

namespace gazebo
{
class LidarPlugin : public RayPlugin
{
public:
  LidarPlugin() = default;

  virtual ~LidarPlugin() = default;

  /**
   * @brief Load the robot controller plugin, overrides the Load from ModelPlugin
   * @param aParent: parent model
   * @param aSdf: the sdf (xml) in the robot model, within the <plugin> element
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

  // Variables
  std::string frameName;
  std::string topicName;
  std::string mLidarDataTopic;
  sensors::RaySensorPtr parentSensor;

  // Gazebo variables
  transport::NodePtr gazeboNode;
  transport::SubscriberPtr gazeboSub;

  // Ros variables
  ros::NodeHandlePtr rosNode;
  ros::Publisher rosPub;
  ros::Publisher mLidarDataPub;
};
GZ_REGISTER_SENSOR_PLUGIN(LidarPlugin)
}  // namespace gazebo