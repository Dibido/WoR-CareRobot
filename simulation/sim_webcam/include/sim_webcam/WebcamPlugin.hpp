
#ifndef PROJECT_WEBCAMPLUGIN_HPP
#define PROJECT_WEBCAMPLUGIN_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <sstream>
#include <string>

namespace gazebo
{
  /**
   * Plugin class for the Camera sensor to pull data from the sensor and put it
   * on a ROS topic
   */
  class WebcamPlugin : public CameraPlugin, GazeboRosCameraUtils
  {
      public:
    /**
     * Default constructor
     */
    WebcamPlugin() = default;

    /**
     * Default destructor
     */
    virtual ~WebcamPlugin() = default;

    /**
     * Gets executed when there is data being published.
     */
    void callback(const sensor_msgs::Image& aMsg);

    /**
     * Setup the sensor plugin. Gets executed when the model (SDF file) gets
     * loaded
     * @param sensor pointer to the sensor
     * @param sdf pointer to the sdf (defined in the model file) element of the
     * sensor
     */
    virtual void Load(sensors::SensorPtr aModel, sdf::ElementPtr aSdf);

  };

} // namespace gazebo

#endif
