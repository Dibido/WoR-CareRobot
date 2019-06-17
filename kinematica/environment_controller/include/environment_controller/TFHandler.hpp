#ifndef TF_HANDLER_H
#define TF_HANDLER_H

#include "environment_controller/EnvironmentConsts.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "environment_controller/Pose.hpp"

namespace environment_controller
{
  /**
   * @brief Helper class for tf2_ros
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
     * @param aPose Pose of the object
     * @param aStatic Boolean that indicaties whether the transform is static or
     * not
     * @param aHeaderFrame
     * @param aChildFrame
     *
     */
    void transform(const Pose& aPose,
                   const bool aStatic,
                   const std::string& aHeaderFrame,
                   const std::string& aChildFrame);

    /**
     * @brief Transforms the fromFrame coordinate system into the toFrame
     * coordinate system
     *
     * @param aFromFrame A collection of coordinates from a single coordinate
     * system
     * @param aToFrame A collection of coordinates from a single coordinate
     * system
     * @return Pose See Pose.hpp
     */
    Pose calculatePosition(const std::string& aFromFrame,
                           const std::string& aToFrame);

      private:
    tf2_ros::TransformBroadcaster mBroadcaster;
    tf2_ros::Buffer mBuffer;
    tf2_ros::TransformListener mTfListener;
  };
} // namespace environment_controller

#endif
