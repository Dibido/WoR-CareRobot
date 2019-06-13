#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <math.h> /* cos and sin */
#include <ros/ros.h>
#include <sim_child/child.hpp>

void gazebo::Child::Load(physics::ModelPtr aParent, sdf::ElementPtr aSdf)
{
  // Store the pointer to the model
  this->mModel = aParent;

  // Loads the config from the sdf file.
  loadConfig(aSdf);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->mUpdateConnection =
      event::Events::ConnectWorldUpdateBegin(std::bind(&Child::onUpdate, this));
  ROS_INFO("Child plugin loaded");
}

void gazebo::Child::onUpdate()
{
  // Apply a linear velocity to the model.
  updateVelocity();
}

void gazebo::Child::updateVelocity()
{
  this->mModel->SetLinearVel(
      ignition::math::Vector3d(calculateXVelocity(), calculateYVelocity(), 0));
}

void gazebo::Child::loadConfig(sdf::ElementPtr aSdf)
{
  mVelocity = loadVelocity(aSdf);
  mAngle = loadAngle(aSdf);
}

double gazebo::Child::loadVelocity(sdf::ElementPtr aSdf)
{
  double velocity;
  if (aSdf->HasElement("velocity"))
  {
    velocity = aSdf->Get<double>("velocity");
    ROS_INFO("Velocity loaded");
  }
  else
  {
    velocity = 0;
    ROS_WARN("Velocity not found. Velocity is set to 0.");
  }

  return velocity;
}

double gazebo::Child::loadAngle(sdf::ElementPtr aSdf)
{
  double angle;
  if (aSdf->HasElement("angle"))
  {
    angle = aSdf->Get<double>("angle");
    ROS_INFO("Angle loaded");
  }
  else
  {
    angle = 0;
    ROS_WARN("Angle not found. Angle is set to 0.");
  }

  return angle;
}

double gazebo::Child::calculateXVelocity()
{
  return mVelocity * cos(mAngle * (M_PI / 180));
}

double gazebo::Child::calculateYVelocity()
{
  return mVelocity * sin(mAngle * (M_PI / 180));
}