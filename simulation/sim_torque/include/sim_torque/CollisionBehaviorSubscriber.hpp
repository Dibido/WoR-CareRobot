#ifndef COLLISION_BEHAVIOR_SUBSCRIBER_HPP
#define COLLISION_BEHAVIOR_SUBSCRIBER_HPP

#include "ICollisionBehavior.hpp"
#include "ros/ros.h"
#include "sim_torque/collision_behavior.h"


namespace sim_torque
{
    class CollisionBehaviorSubscriber : public ICollisionBehavior
    {
    private:
        CollisionBehaviorData mCollisionBehaviorData;
        ros::NodeHandlePtr mNodeHandle;
        ros::Subscriber mSubcriber;
    public:
        CollisionBehaviorSubscriber();
        virtual ~CollisionBehaviorSubscriber() = default;

        virtual void setCollisionBehavior(const sim_torque::CollisionBehaviorData& aCollisionBehaviorData) override;
        void setCollisionBehaviorCallback(const sim_torque::collision_behaviorPtr& aMsg);


    };

}
#endif // COLLISION_BEHAVIOR_SUBSCRIBER_HPP