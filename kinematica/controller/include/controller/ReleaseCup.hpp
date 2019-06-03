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
     * @brief Construct a new Emergency Stop object
     *
     */
    ReleaseCup();
    /**
     * @brief Destroy the Emergency Stop object
     *
     */
    ~ReleaseCup();
    /**
     * @brief entryAction is being called when the ReleaseCup state is being
     * entered.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * ReleaseCup.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext);
    /**
     * @brief exitAction is being called when the ReleaseCup state is being
     * exited.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext);

      private:
    ros::Time mReleaseTime;
  };
} // namespace controller
#endif // RELEASE_CUP_HPP