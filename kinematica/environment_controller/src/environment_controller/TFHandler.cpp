#include "environment_controller/TFHandler.hpp"

TFHandler::TFHandler() : listener(ros::Duration(10))
{
}

TFHandler::~TFHandler()
{
}

void TFHandler::transform(const Position& aPosition, const std::string& aFrame)
{
  tf::Transform lTransform;
  lTransform.setOrigin(
      tf::Vector3(aPosition.x_m(), aPosition.y_m(), aPosition.z_m()));
  tf::Quaternion lQ = tf::Quaternion(aPosition.roll, aPosition.pitch,
                                     aPosition.yaw, position.w);
  q.normalize();
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), GLOBAL_FRAME, aFrame));
}

Position TFHandler::calculatePosition(const std::string& aFromFrame,
                                      const std::string& aToFrame)
{
  Position lNewPos;
  Rotation lNewRot;

  tf::StampedTransform lTransform;
  // Will throw exception if can't parse within time
  ros::Time lNow = ros::Time::now();
  listener.lookupTransform(aToFrame, aFromFrame, ros::Time(0), transform);

  lNewPos.posX = lTransform.getOrigin().getX();
  lNewPos.posY = lTransform.getOrigin().getY();
  lNewPos.posZ = lTransform.getOrigin().getZ();

  // tf::Quaternion lQ = lTransform.getRotation();
  lNewRot.roll_rad() = lTransform.getOrigin().x();
  lNewRot.pitch_rad() = lTransform.getOrigin().y();
  lNewRot.yaw_rad() = lTransform.getOrigin().z();
  lNewRot.quaternion() = lTransform.getOrigin().w();
  return newPos;
}