
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
   * @param _parent: parent model
   * @param _sdf: the sdf (xml) in the robot model, within the <plugin> element
   */
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  /**
   * @brief Callback function of gazebo sensor topic
   *        Publishes a LaserScan message on ros topic @var: topicName
   *        Publishes a LidarData message on ros topic /sensor/lidardata
   * @param _msg: contains the information of the sensor
   */
  void OnScan(ConstLaserScanStampedPtr& _msg);

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