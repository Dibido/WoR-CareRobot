#ifndef POWER_OFF_HPP
#define POWER_OFF_HPP

// Local
#include "Context.hpp"
#include "State.hpp"
namespace controller
{
  /**
   * @class PowerOff
   *
   * @brief PowerOff is the class which represents the PowerOff state.
   *
   */
  class PowerOff : public State
  {
      public:
    /**
     * @brief Construct a new Emergency Stop object
     *
     */
    PowerOff();
    /**
     * @brief Destroy the Emergency Stop object
     *
     */
    ~PowerOff();
    /**
     * @brief entryAction is being called when the PowerOff state is being
     * entered.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * PowerOff.
     *g
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext);
    /**
     * @brief exitAction is being called when the PowerOff state is being
     * exited.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext);
  };
} // namespace controller
#endif // POWER_OFF_HPP