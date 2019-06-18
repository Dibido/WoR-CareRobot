#ifndef STATE_HPP
#define STATE_HPP

#include "Context.hpp"
namespace controller
{
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
     * @param aState State to start with
     */
    State(const State& aState) = delete;

    /**
     * @brief Destructor
     *
     */
    virtual ~State();

    /**
     * @brief entryAction is being called when the State is being entered.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    virtual void entryAction(Context* aContext) = 0;

    /**
     * @brief doActivity is continiously being called while the system is in the
     * State.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    virtual void doActivity(Context* aContext) = 0;

    /**
     * @brief exitAction is being called when the State state is being exited.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    virtual void exitAction(Context* aContext) = 0;

    /**
     * @brief the transition of this state to the next state
     *
     */
    virtual void transition(Context* aContext) = 0;
  };
} // namespace controller
#endif // STATE_HPP