#ifndef INIT_HPP
#define INIT_HPP

// Local
#include "Context.hpp"
#include "State.hpp"

/**
 * @class Init
 *
 * @brief Init is the class which represents the Init state.
 *
 */
class Init : public State
{
    public:
  /**
   * @brief Construct a new Emergency Stop object
   *
   */
  Init();
  /**
   * @brief Destroy the Emergency Stop object
   *
   */
  ~Init();
  /**
   * @brief entryAction is being called when the Init state is being entered.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void entryAction(Context* context);

  /**
   * @brief doActivity is continiously being called while the system is in the
   * Init.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void doActivity(Context* context);
  /**
   * @brief exitAction is being called when the Init state is being exited.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void exitAction(Context* context);
};
#endif // INIT_HPP