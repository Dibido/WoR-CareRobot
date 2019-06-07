#ifndef COLLISION_BEHAVIOR_HANDLER_HPP
#define COLLISION_BEHAVIOR_HANDLER_HPP
#include "Joint.hpp"
#include <vector>

#include "robotcontroller/RobotStopPublisher.hpp"
#include "ros/ros.h"

namespace sim_torque
{
  class CollisionBehaviorHandler
  {
      private:
    void stopRobot();
    ros::NodeHandle node;
    RobotStopPublisher robotStopPublisher(node, "/robot_stop", 1000)

        std::vector<Joint> mJoints;

      public:
    CollisionBehaviorHandler(const std::map<unsigned short, Joint>& aJoints);

    ~CollisionBehaviorHandler() = default;
    void handle();
  };

} // namespace sim_torque

#endif // COLLISION_BEHAVIOR_HANDLER_HPP