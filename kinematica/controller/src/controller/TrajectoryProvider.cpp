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

      ROS_INFO("Configuration: {%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f} [%i]",
               lConfiguration[0], lConfiguration[1], lConfiguration[2],
               lConfiguration[3], lConfiguration[4], lConfiguration[5],
               lConfiguration[6], lConfiguration.result());
      ROS_ASSERT_MSG(
          lConfiguration.result() == true,
          "Could not find configuration for trajectory point %i: "
          "[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]",
          i, lTrajectoryEndEffector.cX_m, lTrajectoryEndEffector.cY_m,
          lTrajectoryEndEffector.cZ_m, lTrajectoryEndEffector.cYaw_rad,
          lTrajectoryEndEffector.cPitch_rad, lTrajectoryEndEffector.cRoll_rad);

      aTrajectory.push(lConfiguration);
    }
    lConfiguration = aContext->configurationProvider()->inverseKinematics(
        aTargetLocation, lConfiguration);

    ROS_ASSERT_MSG(lConfiguration.result() == true, "Goal is not reachable");
    aTrajectory.push(lConfiguration);
  }

  ros::Time TrajectoryProvider::calculateArrivalTime(
      Context* aContext,
      const kinematics::Configuration& aConfiguration)
  {
    double lMaxDeltaTheta = 0;
    for (size_t i = 0; i < aConfiguration.size; ++i)
    {
      if (lMaxDeltaTheta <
          std::abs(aConfiguration[i] - aContext->configuration()[i]))
      {
        lMaxDeltaTheta =
            std::abs(aConfiguration[i] - aContext->configuration()[i]);
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
    planning::Path path = aContext->astar()->search(lStart, lGoal);
    ROS_ASSERT_MSG(path.empty() == false,
                   "No path was found from [%i,%i,%i] to [%i,%i,%i]", lStart.x,
                   lStart.y, lStart.z, lGoal.x, lGoal.y, lGoal.z);
    return path;
  }

} // namespace controller