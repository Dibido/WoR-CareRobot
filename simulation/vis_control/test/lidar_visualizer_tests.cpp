
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

// visualizer
#include <vis_control/LidarVisualizer.hpp>

// message(s)
#include <sensor_msgs/LaserScan.h> // in and out message

using namespace visualization;

BOOST_AUTO_TEST_SUITE(lidar_visualizer_tests)

BOOST_AUTO_TEST_CASE(lidar_visualizer_test_message) {

  // required for ros::time use in the Visualizer class
  ros::Time::init();

  const std::string frameId = "lidar_frame";
  LidarVisualizer visualizer(frameId);

  LidarVisualizer::publishFn_t publishFn =
      [&frameId](const sensor_msgs::LaserScan &message) {
        // Check pre-set values
        BOOST_CHECK_EQUAL(message.range_min, (float)0.1);
        BOOST_CHECK_EQUAL(message.range_max, (float)10.0);

        // Check some values that are set in this visualizer
        BOOST_CHECK_EQUAL(message.header.frame_id, frameId);
      };

  visualizer.setPublishFn(publishFn);

  sensor_msgs::LaserScan message;
  message.range_min = (float)0.1;
  message.range_max = (float)10.0;

  visualizer.callback(message);
}

BOOST_AUTO_TEST_SUITE_END()
