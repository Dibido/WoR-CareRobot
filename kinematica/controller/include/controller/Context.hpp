#ifndef CONTEXT_HPP
#define CONTEXT_HPP

#include "Context.hpp"
#include "State.hpp"
#include <memory>
#include <vector>

class State;
/**
 * @class Context
 *
 * @brief Context is a class which gives the states a interface to the
 * "outside world".
 *
 */
class Context
{
    public:
  /**
   * @brief Construct a new Context object
   *
   */
  Context();

  /**
   * @brief Set the currentState by supplying a shared_ptr to a state.
   *
   * @param state
   */
  void setState(const std::shared_ptr<State>& state);
  /**
   * @brief Run is mthe function which takes care of the handling of the
   * EventQueue and calling the doActivity functions of the different states.
   */
  void run();

  std::shared_ptr<State> currentState;
};
#endif // Context_HPP