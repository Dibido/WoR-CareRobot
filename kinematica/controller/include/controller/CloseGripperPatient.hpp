#ifndef CLOSE_GRIPPER_PATIENT_HPP
#define CLOSE_GRIPPER_PATIENT_HPP

// Local
#include "CloseGripper.hpp"
#include "Context.hpp"
namespace controller
{
  /**
   * @class CloseGripperPatient
   *
   * @brief CloseGripperPatient is the class which represents the
   * CloseGripperPatient state.
   *
   * @details The close gripper state will close the gripper and enter the
   * MoveToDropLocation once the gripper is fully closed.
   *
   */
  class CloseGripperPatient : public CloseGripper
  {
      public:
    /**
     * @brief Construct a new CloseGripperPatient object
     *
     */
    CloseGripperPatient() = default;
    /**
     * @brief Destroy the CloseGripperPatient object
     *
     */
    virtual ~CloseGripperPatient() = default;
    virtual void transition(Context* aContext) override;

      private:
    ros::Time mGripperCloseTime;
  };

} // namespace controller
#endif // CLOSE_GRIPPER_PATIENT_HPP
