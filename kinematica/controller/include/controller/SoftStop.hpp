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
     * @brief Construct a new SoftStop object
     *
     */
    SoftStop();
    /**
     * @brief Destroy the  SoftStop object
     *
     */
    ~SoftStop();
    /**
     * @brief entryAction is being called when the Softstop state is being
     * entered.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext) override;

    /**
     * @brief doActivity is continiously being called while the system is in the
     * Softstop.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext) override;
    /**
     * @brief exitAction is being called when the Softstop state is being
     * exited.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext) override;
  };
} // namespace controller
#endif // SOFT_STOP_HPP