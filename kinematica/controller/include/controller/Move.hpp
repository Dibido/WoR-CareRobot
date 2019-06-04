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
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * Move.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext);
    /**
     * @brief exitAction is being called when the Move state is being
     * exited.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext);

      private:
    TrajectoryProvider mTrajectoryProvider;
    std::queue<kinematics::Configuration> mTrajectory;
    ros::Time mArrivalTime;
  };
} // namespace controller
#endif // MOVE_HPP