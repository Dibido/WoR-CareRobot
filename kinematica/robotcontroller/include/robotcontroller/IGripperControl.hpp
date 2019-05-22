#ifndef ROBOTCONTROLLER_IGRIPPERCONTROL_HPP
#define KINEMATICS_ICONFIGURATIONPROVIDER_HPP

#include "GripperData.hpp"

namespace robotcontroller
{
  /**
   * @brief Interface that is used to control the gripper
   *
   */
  class IGripperControl
  {
      public:
    /**
     * @brief publishes a data object that contains variables that controls the
     * gripper
     * @pre A gripperData struct needs to have been constructed
     * @post The gripperData struct is published. Either the simulated robot or
     * the real-life robot subscribes to this topic to move the gripper
     * @param aGoalPosition
     * @param aCurrentConfiguration
     * @return Configuration
     */
    virtual void
        publishGripper(const robotcontroller::GripperData& gripperData) = 0;
  };
} // namespace robotcontroller

#endif // KINEMATICS_ICONFIGURATIONPROVIDER_HPP