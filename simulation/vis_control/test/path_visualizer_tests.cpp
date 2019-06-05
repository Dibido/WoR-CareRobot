
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

// visualizer
#include <vis_control/PathVisualizer.hpp>

// message(s)
#include <nav_msgs/Path.h>               // out message
#include <path_planner/PathCoordinate.h> // in message

using namespace visualization;

BOOST_AUTO_TEST_SUITE(path_visualizer_tests)

BOOST_AUTO_TEST_CASE(path_visualizer_test_message) {
  // required for ros::time use in the Visualizer class
  ros::Time::init();

  const std::string frameId = "path_frame";
  PathVisualizer visualizer(frameId);

  PathVisualizer::publishFn_t publishFn =
      [&frameId](const nav_msgs::Path &message) {
        BOOST_CHECK_EQUAL(message.poses[0].pose.position.x,
                          static_cast<double>(10));
        BOOST_CHECK_EQUAL(message.poses[0].pose.position.y,
                          static_cast<double>(20));
        BOOST_CHECK_EQUAL(message.poses[0].pose.position.z,
                          static_cast<double>(30));

        BOOST_CHECK_EQUAL(message.poses[0].pose.orientation.x,
                          static_cast<double>(0));
        BOOST_CHECK_EQUAL(message.poses[0].pose.orientation.y,
                          static_cast<double>(0));
        BOOST_CHECK_EQUAL(message.poses[0].pose.orientation.z,
                          static_cast<double>(0));
        BOOST_CHECK_EQUAL(message.poses[0].pose.orientation.w,
                          static_cast<double>(1));

        BOOST_CHECK_EQUAL(message.header.frame_id, frameId);
      };

  visualizer.setPublishFn(publishFn);

  path_planner::PathCoordinate message;
  message.coordinate.x = static_cast<double>(1000);
  message.coordinate.y = static_cast<double>(2000);
  message.coordinate.z = static_cast<double>(3000);

  visualizer.callback(message);
}

BOOST_AUTO_TEST_SUITE_END()
