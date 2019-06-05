#include "environment_controller/TFHandler.hpp"

namespace environment_controller
{

  TFHandler::TFHandler()
  {
    tf2_ros::TransformListener lListener(mBuffer);
  }

  void TFHandler::transform(const Pose& aPose, const std::string& aFrame)
  {
    geometry_msgs::TransformStamped lTransformStamped;

    lTransformStamped.header.stamp = ros::Time::now();
    lTransformStamped.header.frame_id = cGlobalFrame;
    lTransformStamped.child_frame_id = aFrame;
    lTransformStamped.transform.translation.x = aPose.position().x_m();
    lTransformStamped.transform.translation.y = aPose.position().y_m();
    lTransformStamped.transform.translation.z = aPose.position().z_m();
    tf2::Quaternion q = tf2::Quaternion(
        aPose.rotation().roll_rad(), aPose.rotation().pitch_rad(),
        aPose.rotation().yaw_rad(), aPose.rotation().quaternion());

    q.normalize();

    lTransformStamped.transform.rotation.x = q.x();
    lTransformStamped.transform.rotation.y = q.y();
    lTransformStamped.transform.rotation.z = q.z();
    lTransformStamped.transform.rotation.w = q.w();

    mBroadcaster.sendTransform(lTransformStamped);
  }

  Pose TFHandler::calculatePosition(const std::string& aFromFrame,
                                    const std::string& aToFrame)
  {

    geometry_msgs::TransformStamped lTransformStamped;

    ros::Time lNow = ros::Time::now();
    lTransformStamped =
        mBuffer.lookupTransform(aToFrame, aFromFrame, ros::Time(0));

    Position lPosition(lTransformStamped.transform.translation.x,
                       lTransformStamped.transform.translation.y,
                       lTransformStamped.transform.translation.z);
    Rotation lRotation(lTransformStamped.transform.rotation.x,
                       lTransformStamped.transform.rotation.y,
                       lTransformStamped.transform.rotation.z,
                       lTransformStamped.transform.rotation.w);
    Pose lPose(lPosition, lRotation);

    return lPose;
  }

} // namespace environment_controller