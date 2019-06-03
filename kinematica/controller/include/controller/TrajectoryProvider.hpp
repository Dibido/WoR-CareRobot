#ifndef TRAJECTORYPROVIDER_HPP
#define TRAJECTORYPROVIDER_HPP
#include "controller/Context.hpp"
#include "kinematics/Configuration.hpp"
#include <memory>
#include <queue>

namespace controller
{
  class TrajectoryProvider
  {
      public:
    TrajectoryProvider();
    ~TrajectoryProvider() = default;

    /**
     * @brief Create a Trajectory to move from current configuration to target
     * location
     * @pre aTrajectory is empty
     * @pre aTargetLocation is reachable with both planning and kinematics
     * components
     * @post aTrajectory contains all configurations needed to reach target
     * location
     * @param aContext
     * @param aTrajectory The trajectory that is found.
     */
    void createTrajectory(Context* aContext,
                          const kinematics::EndEffector& aTargetLocation,
                          std::queue<kinematics::Configuration>& aTrajectory);
    /**
     * @brief Calculate the time needed to arrive at a configuration when
     * starting from the current configuration
     *
     * @param aContext
     * @param aConfiguration
     * @return ros::Time
     */
    ros::Time calculateArrivalTime(
        const kinematics::Configuration& aConfiguration,
        const kinematics::Configuration& aCurrentConfiguration);

      private:
    /**
     * @brief Find path from current location to goal location
     *
     * @param aContext
     * @param aGoal
     * @return planning::Path
     */
    planning::Path findPath(Context* aContext,
                            const kinematics::EndEffector& aGoal);
  };
} // namespace controller

#endif // TRAJECTORYPROVIDER_HPP
