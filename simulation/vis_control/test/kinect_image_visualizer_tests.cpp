
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

// visualizer
#include <vis_control/KinectImageVisualizer.hpp>

// message(s)
#include <sensor_msgs/Image.h> // in and out message

using namespace visualization;

BOOST_AUTO_TEST_SUITE(kinect_image_visualizer_tests)

BOOST_AUTO_TEST_CASE(kinect_image_image_visualizer_test_message) {

  // required for ros::time use in the Visualizer class
  ros::Time::init();

  const std::string frameId = "kinect_image_frame";
  KinectImageVisualizer visualizer(frameId);

  KinectImageVisualizer::publishFn_t publishFn =
      [&frameId](const sensor_msgs::Image &message) {
        // Check pre-set values
        BOOST_CHECK_EQUAL(message.height, 100);
        BOOST_CHECK_EQUAL(message.width, 200);

        // Check some values that are set in this visualizer
        BOOST_CHECK_EQUAL(message.header.frame_id, frameId);
      };

  visualizer.setPublishFn(publishFn);

  sensor_msgs::Image message;
  message.height = 100;
  message.width = 200;

  visualizer.callback(message);
}

BOOST_AUTO_TEST_SUITE_END()
