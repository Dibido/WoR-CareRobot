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
   */
  class CloseGripper : public State
  {
      public:
    /**
     * @brief Construct a new Emergency Stop object
     *
     */
    CloseGripper();
    /**
     * @brief Destroy the Emergency Stop object
     *
     */
    ~CloseGripper();
    /**
     * @brief entryAction is being called when the CloseGripper state is being
     * entered.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * CloseGripper.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext);
    /**
     * @brief exitAction is being called when the CloseGripper state is being
     * exited.
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