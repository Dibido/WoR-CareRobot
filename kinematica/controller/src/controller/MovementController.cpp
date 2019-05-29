#include "controller/MovementController.hpp"
#include "controller/ControllerConsts.hpp"

namespace controller
{
  MovementController::MovementController()
  {
  }

  void MovementController::createTrajectory(
      Context* aContext,
      const kinematics::EndEffector& aTargetLocation,
      std::queue<kinematics::Configuration>& aTrajectory)
  {
    ROS_ASSERT_MSG(aTrajectory.empty() == true, "Queue is not empty");

    auto start = ros::Time::now();
    ROS_DEBUG("Start Move calculation");

    kinematics::Configuration lConfiguration =
        aContext->configurationProvider()->inverseKinematics(
            aTargetLocation, aContext->configuration());

    ROS_ASSERT_MSG(lConfiguration.result() == true,
                   "Target end effector is unreachable");
    planning::Path requiredPath = findPath(aContext, aTargetLocation);
    lConfiguration = aContext->configuration();
    ROS_DEBUG("Found path, size: %i", requiredPath.size());

    for (std::size_t i = 1; i < requiredPath.size(); ++i)
    {
      kinematics::EndEffector lTrajectoryEndEffector = kinematics::EndEffector(
          static_cast<double>(requiredPath[i].x) / 100,
          static_cast<double>(requiredPath[i].y) / 100,
          static_cast<double>(requiredPath[i].z) / 100, 0, M_PI_2, M_PI_2);

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

  ros::Time MovementController::calculateArrivalTime(
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
      MovementController::findPath(Context* aContext,
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