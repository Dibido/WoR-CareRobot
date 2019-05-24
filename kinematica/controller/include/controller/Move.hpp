#ifndef MOVE_HPP
#define MOVE_HPP

// Local
#include "Context.hpp"
#include "State.hpp"

/**
 * @class Move
 *
 * @brief Move is the class which represents the Move state.
 *
 */
class Move : public State
{
    public:
  /**
   * @brief Construct a new Emergency Stop object
   *
   */
  Move();
  /**
   * @brief Destroy the Emergency Stop object
   *
   */
  ~Move();
  /**
   * @brief entryAction is being called when the Move state is being
   * entered.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void entryAction(Context& context);

  /**
   * @brief doActivity is continiously being called while the system is in the
   * Move.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void doActivity(Context& context);
  /**
   * @brief exitAction is being called when the Move state is being
   * exited.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void exitAction(Context& context);
};
#endif //MOVE_HPP