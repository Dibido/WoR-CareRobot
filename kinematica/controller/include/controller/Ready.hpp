#ifndef READY_HPP
#define READY_HPP

// Local
#include "Context.hpp"
#include "State.hpp"

/**
 * @class Ready
 *
 * @brief Ready is the class which represents the Ready state.
 *
 */
class Ready : public State
{
    public:
  /**
   * @brief Construct a new Emergency Stop object
   *
   */
  Ready();
  /**
   * @brief Destroy the Emergency Stop object
   *
   */
  ~Ready();
  /**
   * @brief entryAction is being called when the Ready state is being
   * entered.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void entryAction(Context* context);

  /**
   * @brief doActivity is continiously being called while the system is in the
   * Ready.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void doActivity(Context* context);
  /**
   * @brief exitAction is being called when the Ready state is being
   * exited.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void exitAction(Context* context);
};
#endif // READY_HPP