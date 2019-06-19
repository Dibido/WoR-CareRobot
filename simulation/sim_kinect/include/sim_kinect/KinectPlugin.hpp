
#ifndef PROJECT_KINECTPLUGIN_HPP
#define PROJECT_KINECTPLUGIN_HPP

#include <gazebo_plugins/gazebo_ros_openni_kinect.h>

#include "environment_controller/IObstacles.hpp"
#include "environment_controller/Object.hpp"
#include "environment_controller/Position.hpp"

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
    virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

    /**
     * @brief the function to implement for the interface
     *
     * @param aObstacles The obstacles that are found
     */
    void passObstacles(
        const environment_controller::Obstacles& aObstacles) override;

      private:
    // ROS
    ros::NodeHandlePtr mRosNode;
    ros::Publisher mAgvPublisher;
  };

} // namespace gazebo

#endif
