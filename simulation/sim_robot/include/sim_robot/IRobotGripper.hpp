#ifndef ROBOT_CONTROLLER_GRIPPER_HPP
#define ROBOT_CONTROLLER_GRIPPER_HPP

#include "robotcontroller_msgs/Gripper.h"

#include <vector>

namespace robot_controller_gripper
{

  /**
   * @brief Interface that is used to control the gripper
   * @pre A gripper control message is published on the /robot_gripper topic.
   * @post The gripper control message is parsed and the gripper is in correct
   * position.
   *
   */
  class IRobotGripper
  {
      public:
    /**
     * @brief parse incoming messages
     * @pre incoming: custom message with width and speedfactor
     * @post outgoing: command with an angle, speed, movement type
     * @param aMsg The control message that we are going to parse
     */

    virtual void
        ParseGripperCallback(const robotcontroller_msgs::GripperPtr& aMsg) = 0;
  };
} // namespace robot_controller_gripper

#endif // ROBOT_CONTROLLER_GRIPPER_HPP