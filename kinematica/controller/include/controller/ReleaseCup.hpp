#ifndef RELEASE_CUP_HPP
#define RELEASE_CUP_HPP

// Local
#include "Context.hpp"
#include "State.hpp"

/**
 * @class ReleaseCup
 *
 * @brief ReleaseCup is the class which represents the ReleaseCup state.
 *
 */
class ReleaseCup : public State
{
    public:
  /**
   * @brief Construct a new Emergency Stop object
   *
   */
  ReleaseCup();
  /**
   * @brief Destroy the Emergency Stop object
   *
   */
  ~ReleaseCup();
  /**
   * @brief entryAction is being called when the ReleaseCup state is being
   * entered.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void entryAction(Context& context);

  /**
   * @brief doActivity is continiously being called while the system is in the
   * ReleaseCup.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void doActivity(Context& context);
  /**
   * @brief exitAction is being called when the ReleaseCup state is being
   * exited.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void exitAction(Context& context);
};
#endif //RELEASE_CUP_HPP