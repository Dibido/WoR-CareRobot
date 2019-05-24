#ifndef STATE_HPP
#define STATE_HPP

#include "Context.hpp"
class Context;
/**
 * @brief State is an abstract/interface class which can be used to create
 * states
 *
 */
class State
{
    public:
  /**
   * @brief Default constructor
   */
  State();

  /**
   * @brief Constructor
   *
   * @param anState State to start with
   */
  State(const State& anState) = delete;

  /**
   * @brief Destructor
   *
   */
  virtual ~State();

  /**
   * @brief entryAction is being called when the State is being entered.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  virtual void entryAction(Context* context) = 0;

  /**
   * @brief doActivity is continiously being called while the system is in the
   * State.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  virtual void doActivity(Context* context) = 0;

  /**
   * @brief exitAction is being called when the State state is being exited.
   *
   * @param context is an object which gives the states a interface to the
   * "outside world".
   */
  virtual void exitAction(Context* context) = 0;
};
#endif // STATE_HPP