#ifndef TF_HANDLER_H
#define TF_HANDLER_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "environment_controller/Position.hpp"

/**
 * @brief Helper class for ros Transform
 *
 */
class TFHandler
{
    public:
  /**
   * @brief Construct a new TFHandler object
   *
   */
  TFHandler();

  /**
   * @brief Destroy the TFHandler object
   *
   */
  virtual ~TFHandler() = default;

  /**
   * @brief Publishes frame information
   *
   * @param position Object that exists of x, y and z coordinates
   * @param frame A collection of coordinates from a single coordinate system
   */
  void transform(const Position& aPosition, const std::string& aFrame);

  /**
   * @brief Transforms fromFrame into toFrame and returns the x, y and z
   * coordinates in the coordinate system of toFrame
   *
   * @param fromFrame A collection of coordinates from a single coordinate
   * system
   * @param toFrame A collection of coordinates from a single coordinate system
   * @return Position Struct that exists of x, y and z coordinates
   */
  Position calculatePosition(const std::string& aFromFrame,
                             const std::string& aToFrame);

    private:
  tf2_ros::TransformBroadcaster mBroadcaster;
  tf2_ros::TransformListener mListener;
};

#endif
