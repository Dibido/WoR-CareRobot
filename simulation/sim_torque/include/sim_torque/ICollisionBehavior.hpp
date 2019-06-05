#ifndef I_COLLISSION_BEHAVIOR_HPP
#define I_COLLISSION_BEHAVIOR_HPP

#include "CollisionBehaviorData.hpp"

namespace sim_torque 
{

    class ICollisionBehavior
    {
    public:
        ICollisionBehavior(/* args */) = default;
        virtual ~ICollisionBehavior() = default;

        virtual void setCollisionBehavior( const sim_torque::CollisionBehaviorData& aCollisionBehaviorData) = 0;
    };
    
   
}

#endif // _COLLISSION_BEHAVIOR_HPP