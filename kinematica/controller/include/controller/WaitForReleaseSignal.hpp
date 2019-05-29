#ifndef WAIT_FOR_RELEASE_SIGNAL
#define WAIT_FOR_RELEASE_SIGNAL

#include "State.hpp"

namespace controller
{
  class Context;
  /**
   * @class WaitForReleaseSignal
   *
   * @brief WaitForReleaseSignal is the class which represents the
   * WaitForReleaseSignal state.
   *
   */
  class WaitForReleaseSignal : public State
  {
      public:
    /**
     * @brief Construct a new Emergency Stop object
     *
     */
    WaitForReleaseSignal();
    /**
     * @brief Destroy the Emergency Stop object
     *
     */
    ~WaitForReleaseSignal() = default;
    /**
     * @brief entryAction is being called when the WaitForReleaseSignal state is
     * being entered.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * WaitForReleaseSignal.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext);
    /**
     * @brief exitAction is being called when the WaitForReleaseSignal state is
     * being exited.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext);
  };
} // namespace controller
#endif // WAIT_FOR_RELEASE_SIGNAL