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
     * @brief Construct a new Emergency Stop object
     *
     */
    WaitForCup();
    /**
     * @brief Destroy the Emergency Stop object
     *
     */
    ~WaitForCup();
    /**
     * @brief entryAction is being called when the WaitForCup state is being
     * entered.
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
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext);
  };
} // namespace controller
#endif // WAIT_FOR_CUP_HPP