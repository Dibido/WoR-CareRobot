#ifndef MOVE_TO_DROP_LOCATION_HPP
#define MOVE_TO_DROP_LOCATION_HPP

#include "Context.hpp"
#include "State.hpp"
#include "controller/TrajectoryProvider.hpp"
#include <memory>
#include <queue>
#include <ros/time.h>
namespace controller
{
  class Context;

  /**
   *
   * @class MoveToDropLocation
   *
   * @brief Moves the arm to the location where the cup has to be dropped.
   *
   * @author Gianni Monteban
   *
   */
  class MoveToDropLocation : public State
  {
      public:
    /**
     * @brief Default constructor
     */
    MoveToDropLocation();

    /**
     * @brief Constructor
     *
     * @param aMoveToDropLocation MoveToDropLocation to start with
     */
    MoveToDropLocation(const MoveToDropLocation& aMoveToDropLocation) = delete;

    /**
     * @brief Destructor
     *
     */
    virtual ~MoveToDropLocation() = default;

    /**
     * @brief entryAction is being called when the MoveToDropLocation is being
     * entered.
     *
     * @details Within the entryAction function the target location is set and
     * the trajectory which the robotarm will take is being created.
     *
     * @param aContext is an object which gives the MoveToDropLocations an
     * interface to the "outside world".
     */
    void entryAction(Context* aContext);

    /**
     * @brief doActivity is continiously being called while the system is in the
     * MoveToDropLocation.
     *
     * @details All different configurations of the trajectory are being executed
     * by the robotarm one after another. Once all configurations are executed
     * the state will transit to the waitForReleaseSignal state.
     *
     * @param aContext is an object which gives the MoveToDropLocations an
     * interface to the "outside world".
     */
    void doActivity(Context* aContext);

    /**
     * @brief exitAction is being called when the MoveToDropLocation
     * MoveToDropLocation is being exited.
     *
     * @details Not used at this moment.
     *
     * @param aContext is an object which gives the MoveToDropLocations an
     * interface to the "outside world".
     */
    void exitAction(Context* aContext);

      private:
    TrajectoryProvider mTrajectoryProvider;
    std::queue<kinematics::Configuration> mTrajectory;
    ros::Time mArrivalTime;
  };
} // namespace controller

#endif // MOVE_TO_DROP_LOCATION_HPP
