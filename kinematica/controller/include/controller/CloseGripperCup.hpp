#ifndef CLOSE_GRIPPER_CUP_HPP
#define CLOSE_GRIPPER_CUP_HPP

// Local
#include "CloseGripper.hpp"
#include "Context.hpp"
namespace controller
{
  /**
   * @class CloseGripperCup
   *
   * @brief CloseGripperCup is the class which represents the
   * CloseGripperCup state.
   *
   * @details The close gripper state will close the gripper and enter the
   * MoveToDropLocation once the gripper is fully closed.
   *
   */
  class CloseGripperCup : public CloseGripper
  {
      public:
    /**
     * @brief Construct a new CloseGripperCup object
     *
     */
    CloseGripperCup() = default;
    /**
     * @brief Destroy the CloseGripperCup object
     *
     */
    virtual ~CloseGripperCup() = default;
    virtual void transition(Context* aContext) override;

      private:
    ros::Time mGripperCloseTime;
  };

} // namespace controller
#endif // CLOSE_GRIPPER_CUP_HPP
