#include "controller/TrajectoryProvider.hpp"
#include "controller/ControllerConsts.hpp"
#include "rng/RandomNumberGenerator.hpp"

namespace controller
{
  TrajectoryProvider::TrajectoryProvider()
  {
  }

  void TrajectoryProvider::createTrajectory(
      Context* aContext,
      const kinematics::EndEffector& aTargetLocation,
      std::queue<kinematics::Configuration>& aTrajectory,
      bool aHoverStart,
      bool aHoverEnd)
  {
    ROS_ASSERT_MSG(aTrajectory.empty() == true, "Queue is not empty");

    planning::Path lRequiredPath =
        findPath(aContext, aTargetLocation, aHoverStart, aHoverEnd);
    kinematics::Configuration lConfiguration = aContext->currentConfiguration();
    ROS_DEBUG("Found path, size: %i", lRequiredPath.size());
    kinematics::Configuration lPreviousConfiguration =
        aContext->currentConfiguration();
    bool lFirstTry = true;
    // Start at 1 because first node is start position
    for (std::size_t i = 1; i < lRequiredPath.size();)
    {
      kinematics::EndEffector lTrajectoryEndEffector = kinematics::EndEffector(
          static_cast<double>(lRequiredPath[i].x) /
              planning::cConversionFromMetersToCentimeters,
          static_cast<double>(lRequiredPath[i].y) /
              planning::cConversionFromMetersToCentimeters,
          static_cast<double>(lRequiredPath[i].z) /
              planning::cConversionFromMetersToCentimeters,
          0, M_PI_2, M_PI_2);

      try
      {
        lConfiguration = aContext->configurationProvider()->inverseKinematics(
            lTrajectoryEndEffector, lConfiguration);
        if (isLogicNextConfiguration(lPreviousConfiguration, lConfiguration) ||
            lFirstTry)
        {
          lPreviousConfiguration = lConfiguration;
          aTrajectory.push(lConfiguration);
          ++i;
          lFirstTry = false;
        }
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("%s", e.what());
      }
      for (size_t j = 0; j < lConfiguration.size; ++j)
      {
        lConfiguration.setTheta(
            j, lConfiguration[j] + rng::RandomNumberGenerator().GenerateInRange(
                                       cMinRandomChange, cMaxRandomChange));
      }
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
    for (size_t i = 1; i < aConfiguration.size; ++i)
    {
      if (lMaxDeltaTheta <
          std::abs(aConfiguration[i] - aContext->currentConfiguration()[i]))
      {
        lMaxDeltaTheta =
            std::abs(aConfiguration[i] - aContext->currentConfiguration()[i]);
      }
    }
    return ros::Time::now() +
           ros::Duration(lMaxDeltaTheta / cJointSpeed_rads / cSpeedFactor);
  }

  planning::Path
      TrajectoryProvider::findPath(Context* aContext,
                                   const kinematics::EndEffector& aGoal,
                                   bool aHoverStart,
                                   bool aHoverEnd)
  {
    kinematics::EndEffector aTargetLocation =
        aContext->configurationProvider()->forwardKinematics(
            aContext->currentConfiguration());
    const long lHoverOffset_cm = static_cast<long>(
        cHoverOffset_m * planning::cConversionFromMetersToCentimeters);

    planning::Vertex lStart(
        static_cast<long>(aTargetLocation.cX_m *
                          planning::cConversionFromMetersToCentimeters),
        static_cast<long>(aTargetLocation.cY_m *
                          planning::cConversionFromMetersToCentimeters),
        static_cast<long>(aTargetLocation.cZ_m *
                          planning::cConversionFromMetersToCentimeters));
    if (aHoverStart)
    {
      lStart.z += lHoverOffset_cm;
    }
    planning::Vertex lGoal(
        static_cast<long>(aGoal.cX_m *
                          planning::cConversionFromMetersToCentimeters),
        static_cast<long>(aGoal.cY_m *
                          planning::cConversionFromMetersToCentimeters),
        static_cast<long>(aGoal.cZ_m *
                          planning::cConversionFromMetersToCentimeters));
    if (aHoverEnd)
    {
      lGoal.z += lHoverOffset_cm;
    }
    planning::Path lPath = aContext->astar()->search(lStart, lGoal);
    ROS_ASSERT_MSG(lPath.empty() == false,
                   "No path was found from [%i,%i,%i] to [%i,%i,%i]", lStart.x,
                   lStart.y, lStart.z, lGoal.x, lGoal.y, lGoal.z);

    // If a path was found and hoverstart and/or hoverand was used, add those
    // values to path
    if (aHoverStart)
    {
      lStart.z -= lHoverOffset_cm;
      lPath.insert(lPath.begin(), lStart);
    }
    if (aHoverEnd)
    {
      lGoal.z -= lHoverOffset_cm;
      lPath.push_back(lGoal);
    }
    return lPath;
  }

  bool TrajectoryProvider::isLogicNextConfiguration(
      const kinematics::Configuration& previousConfiguration,
      const kinematics::Configuration& newConfiguration)
  {
    for (size_t i = 0; i < previousConfiguration.size; ++i)
    {
      double lDifference =
          std::abs(previousConfiguration[i] - newConfiguration[i]);
      if (lDifference > cMaxConfigurationDifference_rad)
      {
        return false;
      }
    }
    return true;
  }
} // namespace controller