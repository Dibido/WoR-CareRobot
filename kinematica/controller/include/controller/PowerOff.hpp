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
     * @param context is an object which gives the states a interface to the
     * "outside world".
     */
    void entryAction(Context* context);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * PowerOff.
     *g
     * @param context is an object which gives the states a interface to the
     * "outside world".
     */
    void doActivity(Context* context);
    /**
     * @brief exitAction is being called when the PowerOff state is being
     * exited.
     *
     * @param context is an object which gives the states a interface to the
     * "outside world".
     */
    void exitAction(Context* context);
  };
} // namespace controller
#endif // POWER_OFF_HPP