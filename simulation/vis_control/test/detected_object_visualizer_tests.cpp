
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

// visualizer
#include <vis_control/DetectedObject.hpp>
#include <vis_control/DetectedObjectVisualizer.hpp>

// message(s)
#include <vision/DetectedObjects.h>    // incomming message type
#include <visualization_msgs/Marker.h> // message inside the MarkerArray message
#include <visualization_msgs/MarkerArray.h> // outgoing message type

using namespace visualization;

BOOST_AUTO_TEST_SUITE(object_visualizer_tests)

    BOOST_AUTO_TEST_CASE(object_visualizer_test_message) {
      ros::Time::init();

      DetectedObject object1(1, 1, 1, 1, 1, 1, 1);

      const std::string frameId = "object_frame";
      DetectedObjectVisualizer visualizer(frameId);

      DetectedObjectVisualizer::publishFn_t publishFn =
              [&frameId, &object1](const visualization_msgs::MarkerArray &message) {
                  visualization_msgs::Marker markerMessage = message.markers[0];

                  BOOST_CHECK_EQUAL(markerMessage.type, visualization_msgs::Marker::CUBE);
                  BOOST_CHECK_EQUAL(markerMessage.action,
                                    visualization_msgs::Marker::ADD);

                  BOOST_CHECK_EQUAL(markerMessage.pose.position.x, object1.getX());
                  BOOST_CHECK_EQUAL(markerMessage.pose.position.y, object1.getY());
                  BOOST_CHECK_EQUAL(markerMessage.pose.position.z, (object1.getZ() - 1) / 2);

                  BOOST_CHECK_EQUAL(markerMessage.pose.orientation.x,
                                    static_cast<double>(0.0));
                  BOOST_CHECK_EQUAL(markerMessage.pose.orientation.y,
                                    static_cast<double>(0.0));
                  BOOST_CHECK_EQUAL(markerMessage.pose.orientation.z,
                                    static_cast<double>(0.0));
                  BOOST_CHECK_EQUAL(markerMessage.pose.orientation.w,
                                    static_cast<double>(1.0));

                  BOOST_CHECK_EQUAL(markerMessage.scale.x, object1.getWidth());
                  BOOST_CHECK_EQUAL(markerMessage.scale.y, object1.getHeight());
                  BOOST_CHECK_EQUAL(markerMessage.scale.z, object1.getZ() + 1);

                  BOOST_CHECK_EQUAL(markerMessage.color.r, static_cast<float>(0.0));
                  BOOST_CHECK_EQUAL(markerMessage.color.g, static_cast<float>(1.0));
                  BOOST_CHECK_EQUAL(markerMessage.color.b, static_cast<float>(0.0));
                  BOOST_CHECK_EQUAL(markerMessage.color.a, static_cast<float>(1.0));

                  BOOST_CHECK_EQUAL(message.markers[0].header.frame_id, frameId);
              };

      visualizer.setPublishFn(publishFn);

      vision::DetectedObjects message;
      geometry_msgs::Point positionMessage;

      positionMessage.x = object1.getX() - (object1.getWidth() / 2);
      positionMessage.y = object1.getY() - (object1.getHeight() / 2);
      positionMessage.z = object1.getZ() + (object1.getDepth() / 2);

      message.position.push_back(positionMessage);
      message.depth.push_back(object1.getDepth());
      message.height.push_back(object1.getHeight());
      message.width.push_back(object1.getWidth());
      message.timestamp.data = ros::Time::now();

      visualizer.callback(message);
    }

    BOOST_AUTO_TEST_CASE(object_visualizer_test_clear_message) {
      // TODO
    }

    BOOST_AUTO_TEST_CASE(object_visualizer_test_no_clear_message) {
      // TODO
    }

BOOST_AUTO_TEST_SUITE_END()
