
#ifndef PROJECT_KINECTPLUGIN_HPP
#define PROJECT_KINECTPLUGIN_HPP

#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/gazebo_ros_openni_kinect.h>
#include <ros/subscribe_options.h>
#include <string>
#include <thread>

#include "environment_controller/IObstacles.hpp"
#include "environment_controller/Object.hpp"
#include "environment_controller/Position.hpp"
#include <kinematica_msgs/Object.h>
#include <kinematica_msgs/Obstacles.h>

namespace gazebo
{
  /**
   * Plugin class for the Kinect sensor to pull data from the sensor and put it
   * on a ROS topic
   */
  class KinectPlugin : public GazeboRosOpenniKinect,
                       public environment_controller::IObstacles
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
     * Setup the sensor plugin. Gets executed when the model (SDF file) gets
     * loaded
     * @param sensor pointer to the sensor
     * @param sdf pointer to the sdf (defined in the model file) element of the
     * sensor
     */
    virtual void Load(sensors::SensorPtr aSensor, sdf::ElementPtr aSdf);

    /**
     * @brief the function to implement for the interface
     *
     * @param aObstacles The obstacles that are found
     */
    void passObstacles(
        const environment_controller::Obstacles& aObstacles) override;

    /**
     * Gets executed when there is data being published.
     *
     */
    void callback(const sensor_msgs::PointCloud2ConstPtr& aMsg);

      private:
    // ROS
    ros::NodeHandlePtr mRosNode;
    ros::Publisher mKinectPublisher;
    ros::CallbackQueue mRosQueue;
    std::thread mRosQueueThread;
    ros::Subscriber mRosSub;

    /**
     * @brief Handles incoming ros messages on a separate thread when a new
     * message is available handle it using the callback
     */
    void QueueThread();
  };

} // namespace gazebo

#endif
