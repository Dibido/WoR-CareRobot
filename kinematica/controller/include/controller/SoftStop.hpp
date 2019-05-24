#ifndef SOFT_STOP_HPP
#define SOFT_STOP_HPP

// Local
#include "Context.hpp"
#include "State.hpp"
namespace controller
{
  /**
   * @class Softstop
   *
   * @brief Softstop is the class which represents the Softstop state.
   *
   */
  class SoftStop : public State
  {
      public:
    /**
     * @brief Construct a new Emergency Stop object
     *
     */
    SoftStop();
    /**
     * @brief Destroy the Emergency Stop object
     *
     */
    ~SoftStop();
    /**
     * @brief entryAction is being called when the Softstop state is being
     * entered.
     *
     * @param context is an object which gives the states a interface to the
     * "outside world".
     */
    void entryAction(Context* context);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * Softstop.
     *
     * @param context is an object which gives the states a interface to the
     * "outside world".
     */
    void doActivity(Context* context);
    /**
     * @brief exitAction is being called when the Softstop state is being
     * exited.
     *
     * @param context is an object which gives the states a interface to the
     * "outside world".
     */
    void exitAction(Context* context);
  };
} // namespace controller
#endif // SOFT_STOP_HPP