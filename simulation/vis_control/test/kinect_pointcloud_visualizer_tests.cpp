
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

// visualizer
#include <vis_control/KinectPointsVisualizer.hpp>

// message(s)
#include <sensor_msgs/PointCloud2.h> // in and out message

using namespace visualization;

BOOST_AUTO_TEST_SUITE(kinect_pointcloud_visualizer_tests)

BOOST_AUTO_TEST_CASE(kinect_pointcloud_image_visualizer_test_message) {

  // required for ros::time use in the Visualizer class
  ros::Time::init();

  const std::string frameId = "kinect_pointcloud_frame";
  KinectPointsVisualizer visualizer(frameId);

  KinectPointsVisualizer::publishFn_t publishFn =
      [&frameId](const sensor_msgs::PointCloud2 &message) {
        // Check pre-set values
        BOOST_CHECK_EQUAL(message.height, 100);
        BOOST_CHECK_EQUAL(message.width, 200);

        // Check some values that are set in this visualizer
        BOOST_CHECK_EQUAL(message.header.frame_id, frameId);
      };

  visualizer.setPublishFn(publishFn);

  sensor_msgs::PointCloud2 message;
  message.height = 100;
  message.width = 200;

  visualizer.callback(message);
}

BOOST_AUTO_TEST_SUITE_END()
