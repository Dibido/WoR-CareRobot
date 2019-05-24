#ifndef CLOSE_GRIPPER_HPP
#define CLOSE_GRIPPER_HPP

// Local
#include "Context.hpp"
#include "State.hpp"

/**
 * @class CloseGripper
 *
 * @brief CloseGripper is the class which represents the CloseGripper state.
 *
 */
class CloseGripper : public State
{
    public:
  /**
   * @brief Construct a new Emergency Stop object
   *
   */
  CloseGripper();
  /**
   * @brief Destroy the Emergency Stop object
   *
   */
  ~CloseGripper();
  /**
   * @brief entryAction is being called when the CloseGripper state is being
   * entered.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void entryAction(Context* context);

  /**
   * @brief doActivity is continiously being called while the system is in the
   * CloseGripper.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void doActivity(Context* context);
  /**
   * @brief exitAction is being called when the CloseGripper state is being
   * exited.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  void exitAction(Context* context);
};
#endif // CLOSE_GRIPPER_HPP