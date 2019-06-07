#include "sim_torque/CollisionBehaviorModel.hpp"

void gazebo::CollisionBehaviorModel::Load(physics::ModelPtr aParent,
                                          sdf::ElementPtr aSdf)
{
  // Store the pointer to the model
  this->mModel = aParent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CollisionBehaviorModel::OnUpdate, this));
}

void gazebo::CollisionBehaviorModel::OnUpdate()
{
  for (auto& lJoint : this->mModel->GetJoints())
  {
    std::cout << std::endl;

    std::cout << lJoint->GetName() << std::endl;
    std::cout << lJoint->GetType() << std::endl;
    std::cout << "0: " << lJoint->GetVelocity(0) << std::endl;
    std::cout << "1: " << lJoint->GetVelocity(1) << std::endl;
    std::cout << "2: " << lJoint->GetVelocity(2) << std::endl;
    std::cout << "3: " << lJoint->GetVelocity(3) << std::endl;
    std::cout << "4: " << lJoint->GetVelocity(4) << std::endl;
    std::cout << "5: " << lJoint->GetVelocity(5) << std::endl;
    std::cout << "6: " << lJoint->GetVelocity(6) << std::endl;
  }
}