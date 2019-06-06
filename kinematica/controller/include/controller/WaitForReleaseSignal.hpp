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
   * @details The class will wait for a release message and wait the time
   * specified in the message before transiting to the ReleaseCup state.
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
     * @details At this point nothing is being done in the entry of the
     * WaitForReleaseSignal state.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * WaitForReleaseSignal.
     *
     * @details Within the doActivity function will be blocking untill a
     * ReleaseMessage has been received. Once a release message has been
     * received we will wait the time specified in the message.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext);
    /**
     * @brief exitAction is being called when the WaitForReleaseSignal state is
     * being exited.
     *
     * @details At this moment nothing is done in the exitAction.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext);
  };
} // namespace controller
#endif // WAIT_FOR_RELEASE_SIGNAL
