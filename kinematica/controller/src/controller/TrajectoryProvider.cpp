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
      std::queue<kinematics::Configuration>& aTrajectory,
      bool aHoverStart,
      bool aHoverEnd)
  {
    ROS_ASSERT_MSG(aTrajectory.empty() == true, "Queue is not empty");

    planning::Path lRequiredPath =
        findPath(aContext, aTargetLocation, aHoverStart, aHoverEnd);
    kinematics::Configuration lConfiguration = aContext->configuration();
    ROS_DEBUG("Found path, size: %i", lRequiredPath.size());

    for (std::size_t i = 0; i < lRequiredPath.size(); ++i)
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

      ROS_DEBUG(
          "TrajectoryProvider, node [%i] Configuration: "
          "{%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f} [%i]",
          i, lConfiguration[0], lConfiguration[1], lConfiguration[2],
          lConfiguration[3], lConfiguration[4], lConfiguration[5],
          lConfiguration[6], lConfiguration.result());
      ROS_ASSERT_MSG(
          lConfiguration.result() == true,
          "Could not find configuration for trajectory point %i: "
          "[%.5f,%.5f,%.5f,%.5f,%.5f,%.5f]",
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
    for (size_t i = 1; i < aConfiguration.size; ++i)
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
                                   const kinematics::EndEffector& aGoal,
                                   bool aHoverStart,
                                   bool aHoverEnd)
  {
    kinematics::EndEffector aTargetLocation =
        aContext->configurationProvider()->forwardKinematics(
            aContext->configuration());
    long lHoverOffset_cm = static_cast<long>(
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

    // If a path was found and hoverstart and/or hoverand was used, add that
    // value to the path

    std::stringstream ss;

    for (std::size_t i = 0; i < lPath.size(); ++i)
    {
      ss << "[" << lPath[i].x << "," << lPath[i].y << "," << lPath[i].z << "]"
         << std::endl;
    }

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

    for (std::size_t i = 0; i < lPath.size(); ++i)
    {
      ss << "[" << lPath[i].x << "," << lPath[i].y << "," << lPath[i].z << "]"
         << std::endl;
    }

    ROS_INFO_STREAM(ss.str());
    return lPath;
  }

} // namespace controller