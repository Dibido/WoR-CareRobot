#include "sim_torque/CollisionBehaviorSubscriber.hpp"
#include "sim_torque/CollisionBehaviorData.hpp"
#include <string>

const std::string cSetCollisionBehaviorTopic = "/set_collision_behavior";

sim_torque::CollisionBehaviorSubscriber::CollisionBehaviorSubscriber()
{
  mNodeHandle = std::make_unique<ros::NodeHandle>("sim_torque_plugin");
  mSubcriber = mNodeHandle->subscribe(
      "/set_collision_behavior", 1000,
      &sim_torque::CollisionBehaviorSubscriber::setCollisionBehaviorCallback,
      this);
}

void sim_torque::CollisionBehaviorSubscriber::setCollisionBehavior(
    const sim_torque::CollisionBehaviorData& aCollisionBehaviorData)
{
  mCollisionBehaviorData = aCollisionBehaviorData;
}

void sim_torque::CollisionBehaviorSubscriber::setCollisionBehaviorCallback(
    const sim_torque::collision_behaviorPtr& aMsg)
{
  sim_torque::CollisionBehaviorData lCollisionBehaviorData(
      aMsg->lower_torque_thresholds_acceleration,
      aMsg->upper_torque_thresholds_acceleration,
      aMsg->lower_torque_thresholds_nominal,
      aMsg->upper_torque_thresholds_nominal,
      aMsg->lower_force_thresholds_acceleration,
      aMsg->upper_force_thresholds_acceleration,
      aMsg->lower_force_thresholds_nominal,
      aMsg->upper_force_thresholds_nominal);
  setCollisionBehavior(lCollisionBehaviorData);
}