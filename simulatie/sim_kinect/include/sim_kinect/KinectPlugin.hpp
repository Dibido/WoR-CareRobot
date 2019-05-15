
#ifndef PROJECT_KINECTPLUGIN_HPP
#define PROJECT_KINECTPLUGIN_HPP

#include <gazebo_plugins/gazebo_ros_openni_kinect.h>

namespace gazebo
{
/**
 * Plugin class for the Kinect sensor to pull data from the sensor and put it on a ROS topic
 */
class KinectPlugin : public GazeboRosOpenniKinect
{
public:
  /**
   * Default constructor
   */
  KinectPlugin() = default;

  /**
   * Default destructor
   */
  virtual ~KinectPlugin() = default;

  /**
   * Setup the sensor plugin. Gets executed when the model (SDF file) gets loaded
   * @param sensor pointer to the sensor
   * @param sdf pointer to the sdf (defined in the model file) element of the sensor
   */
  virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);
};

}  // namespace gazebo

#endif
