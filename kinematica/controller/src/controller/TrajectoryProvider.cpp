#include "controller/TrajectoryProvider.hpp"
#include "controller/ControllerConsts.hpp"

namespace controller
{
  TrajectoryProvider::TrajectoryProvider()
  {
  }

  void TrajectoryProvider::createTrajectory(
      Context* aContext,
      const kinematics::EndEffector& aTargetLocation,
      std::queue<kinematics::Configuration>& aTrajectory)
  {
    ROS_ASSERT_MSG(aTrajectory.empty() == true, "Queue is not empty");

    planning::Path lRequiredPath = findPath(aContext, aTargetLocation);
    kinematics::Configuration lConfiguration = aContext->configuration();
    ROS_DEBUG("Found path, size: %i", lRequiredPath.size());

    // Start at 1 because first node is start position
    for (std::size_t i = 1; i < lRequiredPath.size(); ++i)
    {
      kinematics::EndEffector lTrajectoryEndEffector = kinematics::EndEffector(
          static_cast<double>(lRequiredPath[i].x) /
              planning::cConversionFromMetersToCentimeters,
          static_cast<double>(lRequiredPath[i].y) /
              planning::cConversionFromMetersToCentimeters,
          static_cast<double>(lRequiredPath[i].z) /
              planning::cConversionFromMetersToCentimeters,
          0, M_PI_2, M_PI_2);

      lConfiguration = aContext->configurationProvider()->inverseKinematics(
          lTrajectoryEndEffector, lConfiguration);

      ROS_ASSERT_MSG(lConfiguration.result() == true,
                     "Could not find configuration for trajectory point %i", i);

      aTrajectory.push(lConfiguration);
    }
    lConfiguration = aContext->configurationProvider()->inverseKinematics(
        aTargetLocation, lConfiguration);

    ROS_ASSERT_MSG(lConfiguration.result() == true, "Goal is not reachable");
    aTrajectory.push(lConfiguration);
  }

  ros::Time TrajectoryProvider::calculateArrivalTime(
      const kinematics::Configuration& aConfiguration,
      const kinematics::Configuration& aCurrentConfiguration)
  {
    double lMaxDeltaTheta = 0;
    for (size_t i = 0; i < aConfiguration.size; ++i)
    {
      if (lMaxDeltaTheta <
          std::abs(aConfiguration[i] - aCurrentConfiguration[i]))
      {
        lMaxDeltaTheta = std::abs(aConfiguration[i] - aCurrentConfiguration[i]);
      }
    }
    return ros::Time::now() +
           ros::Duration(lMaxDeltaTheta / cJointSpeed_rads / cSpeedFactor);
  }

  planning::Path
      TrajectoryProvider::findPath(Context* aContext,
                                   const kinematics::EndEffector& aGoal)
  {
    kinematics::EndEffector aTargetLocation =
        aContext->configurationProvider()->forwardKinematics(
            aContext->configuration());

    planning::Vertex lStart(
        static_cast<long>(aTargetLocation.cX_m *
                          planning::cConversionFromMetersToCentimeters),
        static_cast<long>(aTargetLocation.cY_m *
                          planning::cConversionFromMetersToCentimeters),
        static_cast<long>(aTargetLocation.cZ_m *
                          planning::cConversionFromMetersToCentimeters));

    planning::Vertex lGoal(
        static_cast<long>(aGoal.cX_m *
                          planning::cConversionFromMetersToCentimeters),
        static_cast<long>(aGoal.cY_m *
                          planning::cConversionFromMetersToCentimeters),
        static_cast<long>(aGoal.cZ_m *
                          planning::cConversionFromMetersToCentimeters));

    return aContext->astar()->search(lStart, lGoal);
  }

} // namespace controller