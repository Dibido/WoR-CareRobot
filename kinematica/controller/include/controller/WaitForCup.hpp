#ifndef WAIT_FOR_CUP_HPP
#define WAIT_FOR_CUP_HPP

// Local
#include "Context.hpp"
#include "State.hpp"
namespace controller
{
  /**
   * @class WaitForCup
   *
   * @brief WaitForCup is the class which represents the WaitForCup state.
   *
   */
  class WaitForCup : public State
  {
      public:
    /**
     * @brief Construct a new WaitForCup object
     *
     */
    WaitForCup();
    /**
     * @brief Destroy the WaitForCup object
     *
     */
    ~WaitForCup();
    /**
     * @brief entryAction is being called when the WaitForCup state is being
     * entered.
     *
     * @details The entryAction will calculate the amount of time it will take
     * for the cup to arrive. After calculating the time it the thread will be
     * put to sleep for the time it takes for the cup to arrive minus a
     * waittime.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * WaitForCup.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext);
    /**
     * @brief exitAction is being called when the WaitForCup state is being
     * exited.
     *
     * @details Check whether the time for the cup to arrive has passed. If the
     * time has passed a transition will be made to the CloseGripper state.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext);
  };
} // namespace controller
#endif // WAIT_FOR_CUP_HPP