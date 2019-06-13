#ifndef RELEASE_CUP_HPP
#define RELEASE_CUP_HPP

// Local
#include "Context.hpp"
#include "State.hpp"
namespace controller
{
  /**
   * @class ReleaseCup
   *
   * @brief ReleaseCup is the class which represents the ReleaseCup state.
   *
   */
  class ReleaseCup : public State
  {
      public:
    /**
     * @brief Construct a new ReleaseCup object
     *
     */
    ReleaseCup();
    /**
     * @brief Destroy the ReleaseCup object
     *
     */
    ~ReleaseCup();
    /**
     * @brief entryAction is being called when the ReleaseCup state is being
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
     * ReleaseCup.
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
     * @brief exitAction is being called when the ReleaseCup state is being
     * exited.
     *
     * @details At this moment the exitAction is not used.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext) override;

      private:
    ros::Time mReleaseTime;
  };
} // namespace controller
#endif // RELEASE_CUP_HPP