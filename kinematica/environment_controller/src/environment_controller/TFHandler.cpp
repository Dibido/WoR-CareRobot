#include "environment_controller/TFHandler.hpp"

namespace environment_controller
{

  TFHandler::TFHandler() : mTfListener(mBuffer)
  {
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

    tf2::Quaternion lQ =
        tf2::Quaternion(aPose.rotation().x(), aPose.rotation().y(),
                        aPose.rotation().z(), aPose.rotation().w());

    lQ.normalize();

    lTransformStamped.transform.rotation.x = lQ.x();
    lTransformStamped.transform.rotation.y = lQ.y();
    lTransformStamped.transform.rotation.z = lQ.z();
    lTransformStamped.transform.rotation.w = lQ.w();

    mBroadcaster.sendTransform(lTransformStamped);
  }

  Pose TFHandler::calculatePosition(const std::string& aFromFrame,
                                    const std::string& aToFrame)
  {

    geometry_msgs::TransformStamped lTransformStamped;

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