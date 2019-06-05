#ifndef CLOSE_GRIPPER_HPP
#define CLOSE_GRIPPER_HPP

// Local
#include "Context.hpp"
#include "State.hpp"
namespace controller
{
  /**
   * @class CloseGripper
   *
   * @brief CloseGripper is the class which represents the CloseGripper state.
   *
   * @details The close gripper state will close the gripper and enter the
   * MoveToDropLocation once the gripper is fully closed.
   *
   */
  class CloseGripper : public State
  {
      public:
    /**
     * @brief Construct a new CloseGripper object
     *
     */
    CloseGripper();
    /**
     * @brief Destroy the CloseGripper object
     *
     */
    ~CloseGripper();
    /**
     * @brief entryAction is being called when the CloseGripper state is being
     * entered.
     *
     * @details The entryAction will calculate the time it takes to close the
     * gripper. It will furthermore start closing the gripper. After telling the
     * gripper to close the thread will be put to sleep for the time
     * it takes to close the gripper minus a waittime.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * CloseGripper.
     *
     * @details The doActivity function will check whether the time it takes to
     * close the gripper has passed. Once the time it took to close the gripper
     * has passed it will transit to the MoveToDropLocation state.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext);
    /**
     * @brief exitAction is being called when the CloseGripper state is being
     * exited.
     *
     * @details At this moment the exitAction is not used.
     * 
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext);

      private:
    ros::Time mGripperCloseTime;
  };

} // namespace controller
#endif // CLOSE_GRIPPER_HPP
