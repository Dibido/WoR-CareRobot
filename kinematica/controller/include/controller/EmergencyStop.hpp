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
     * @param context is an object which gives the states a interface to the
     * "outside world".
     */
    void entryAction(Context* context);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * EmergencyStop.
     *
     * @param context is an object which gives the states a interface to the
     * "outside world".
     */
    void doActivity(Context* context);
    /**
     * @brief exitAction is being called when the EmergencyStop state is being
     * exited.
     *
     * @param context is an object which gives the states a interface to the
     * "outside world".
     */
    void exitAction(Context* context);
  };
} // namespace controller
#endif // EMERGENCY_STOP_HPP