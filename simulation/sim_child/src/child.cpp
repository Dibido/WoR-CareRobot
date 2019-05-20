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

  if (aSdf->HasElement("velocity"))
  {
    mVelocity = aSdf->Get<double>("velocity");
    ROS_INFO("Velocity loaded");
  }

  if (aSdf->HasElement("angle"))
  {
    mAngle = aSdf->Get<double>("angle");
    ROS_INFO("Angle loaded");
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->mUpdateConnection =
      event::Events::ConnectWorldUpdateBegin(std::bind(&Child::onUpdate, this));
  ROS_INFO("Child plugin loaded");
}

void gazebo::Child::onUpdate()
{
  // Apply a small linear velocity to the model.
  this->mModel->SetLinearVel(
      ignition::math::Vector3d(mVelocity * cos(mAngle * (M_PI / 180)),
                               mVelocity * sin(mAngle * (M_PI / 180)), 0));
}
