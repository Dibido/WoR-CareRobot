#include <sim_child/child.hpp>
#include <functional>
#include <math.h>       /* cos and sin */
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

#define PI 3.14159265

void gazebo::Child::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->mModel = _parent;

  if (_sdf->HasElement("velocity"))
  {
    mVelocity = _sdf->Get<double>("velocity");
    ROS_INFO("Velocity loaded");  
  }

  if (_sdf->HasElement("angle"))
  {
    mAngle = _sdf->Get<double>("angle");
    ROS_INFO("Angle loaded");  
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&Child::onUpdate, this));
  ROS_INFO("Child plugin loaded");
}


void gazebo::Child::onUpdate()
{
  // Apply a small linear velocity to the model.
  this->mModel->SetLinearVel(ignition::math::Vector3d(mVelocity * cos( mAngle * ( PI / 180 )), mVelocity * sin( mAngle * ( PI / 180 )), 0));
}
