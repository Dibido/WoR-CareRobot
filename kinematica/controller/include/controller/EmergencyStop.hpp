#ifndef EMERGENCY_STOP_HPP
#define EMERGENCY_STOP_HPP

// Local
#include "Context.hpp"
#include "State.hpp"
namespace controller
{
  /**
   * @class EmergencyStop
   *
   * @brief EmergencyStop is the class which represents the EmergencyStop state.
   *
   */
  class EmergencyStop : public State
  {
      public:
    /**
     * @brief Construct a new Emergency Stop object
     *
     */
    EmergencyStop();
    /**
     * @brief Destroy the Emergency Stop object
     *
     */
    ~EmergencyStop();
    /**
     * @brief entryAction is being called when the EmergencyStop state is being
     * entered.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * EmergencyStop.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext);
    /**
     * @brief exitAction is being called when the EmergencyStop state is being
     * exited.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext);
  };
} // namespace controller
#endif // EMERGENCY_STOP_HPP