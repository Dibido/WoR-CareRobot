#include "sim_kinect/KinectPlugin.hpp"

namespace gazebo
{
  const std::string cKinectPublishTopic = "/sensor/kinect/obstacles";
  const std::string cImgSubscribeTopic = "/sensor/kinect/points";
  const std::string cKinect_Nh = "Kinect_plugin_NH";

  void callback(const sensor_msgs::PointCloud2ConstPtr aMsg)
  {
  }

  void KinectPlugin::Load(sensors::SensorPtr aSensor, sdf::ElementPtr aSdf)
  {
    GazeboRosOpenniKinect::Load(aSensor, aSdf);

    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "Kinect_plugin",
                ros::init_options::NoSigintHandler);
    }

    mRosNode = std::make_unique<ros::NodeHandle>(cKinect_Nh);

    mKinectPublisher = mRosNode->advertise<kinematica_msgs::Obstacles>(
        gazebo::cKinectPublishTopic, 1);

    mRosNode->subscribe(gazebo::cImgSubscribeTopic, 10,
                        &KinectPlugin::callback);
    // ros::SubscribeOptions so =
    //     ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
    //         gazebo::cImgSubscribeTopic, 1,
    //         boost::bind(&KinectPlugin::callback, this, _1), ros::VoidPtr(),
    //         &this->mRosQueue);
    // mRosSub = mRosNode->subscribe(so);

    // this->mRosQueueThread =
    //     std::thread(std::bind(&KinectPlugin::QueueThread, this));
  }

  void KinectPlugin::passObstacles(
      const environment_controller::Obstacles& aObstacles)
  {
  }

  void KinectPlugin::QueueThread()
  {
    static const double timeout = 0.01;
    while (this->mRosNode->ok())
    {
      this->mRosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  GZ_REGISTER_SENSOR_PLUGIN(KinectPlugin);
} // namespace gazebo
