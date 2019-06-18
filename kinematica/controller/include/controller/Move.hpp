#ifndef MOVE_HPP
#define MOVE_HPP

#include "Context.hpp"
#include "State.hpp"
#include "controller/TrajectoryProvider.hpp"
#include <memory>
#include <queue>
#include <ros/time.h>

namespace controller
{
  /**
   * @class Move
   *
   * @brief Moves the arm to the location where the cup has grabbed
   *
   * @author Martijn Vogelaar
   *
   */
  class Move : public State
  {
      public:
    /**
     * @brief Construct a new Move object
     *
     */
    Move();
    /**
     * @brief Destroy the Move object
     *
     */
    ~Move();
    /**
     * @brief entryAction is being called when the Move state is being
     * entered.
     *
     * @details Within the entryAction function the target location is set and
     * the trajectory which the robotarm will take is being created.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    virtual void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * Move.
     *
     * @details All different configurations of the trajectory are being
     * executed by the robotarm one after another. Once the last configuration
     * has been executed the state will transit to the WaitForCup state.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext) override;
    /**
     * @brief exitAction is being called when the Move state is being
     * exited.
     *
     * @details Not used at this moment.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext) override;

    /**
     * @brief implements the transition to the next state
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    virtual void transition(Context* aContext);

      protected:
    TrajectoryProvider mTrajectoryProvider;
    std::queue<kinematics::Configuration> mTrajectory;
    ros::Time mArrivalTime;
  };
} // namespace controller
#endif // MOVE_HPP
