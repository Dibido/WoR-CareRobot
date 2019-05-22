
#ifndef PROJECT_WEBCAMPLUGIN_HPP
#define PROJECT_WEBCAMPLUGIN_HPP

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <ros/subscribe_options.h>
#include <sstream>
#include <string>
#include <thread>

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
    void callback(const sensor_msgs::ImageConstPtr aMsg);

    /**
     * Setup the sensor plugin. Gets executed when the model (SDF file) gets
     * loaded
     * @param sensor pointer to the sensor
     * @param sdf pointer to the sdf (defined in the model file) element of the
     * sensor
     */
    virtual void Load(sensors::SensorPtr aModel, sdf::ElementPtr aSdf);

      private:
   
    std::unique_ptr<ros::NodeHandle> mRosNode;
    ros::Subscriber mRosSub;
    ros::Publisher mWebcamPublisher;
    ros::CallbackQueue mRosQueue;
    std::thread mRosQueueThread;
    void QueueThread();
  };

} // namespace gazebo

#endif
