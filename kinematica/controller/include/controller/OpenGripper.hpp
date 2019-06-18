#ifndef OPEN_GRIPPER_HPP
#define OPEN_GRIPPER_HPP

// Local
#include "Context.hpp"
#include "State.hpp"
namespace controller
{
  /**
   * @class OpenGripper
   *
   * @brief OpenGripper is the class which represents the OpenGripper state.
   *
   */
  class OpenGripper : public State
  {
      public:
    /**
     * @brief Construct a new OpenGripper object
     *
     */
    OpenGripper();
    /**
     * @brief Destroy the OpenGripper object
     *
     */
    ~OpenGripper();
    /**
     * @brief entryAction is being called when the OpenGripper state is being
     * entered.
     *
     * @details The entryAction will calculate the time it takes to open the
     * gripper. It will furthermore start opening the gripper. After telling the
     * gripper to open the thread will be put to sleep for the time
     * it takes to open the gripper minus a waittime.
     *
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext) override;

    /**
     * @brief doActivity is continiously being called while the system is in the
     * OpenGripper.
     *
     * @details The doActivity function will check whether the time it
     * takes to open the gripper has passed. Once the time it took to open the
     * gripper it will transit to the Ready state.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext) override;
    /**
     * @brief exitAction is being called when the OpenGripper state is being
     * exited.
     *
     * @details At this moment the exitAction is not used.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext) override;

    virtual void transition(Context* aContext);

      private:
    ros::Time mReleaseTime;
  };
} // namespace controller
#endif // OPEN_GRIPPER_HPP