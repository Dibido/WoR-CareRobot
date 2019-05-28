#include "environment_controller/EnvironmentController.hpp"
#include "controller/EmergencyStop.hpp"
#include "environment_controller/SafetyController.hpp"

#include <thread>
namespace environment_controller
{
  EnvironmentController::EnvironmentController(
      const std::shared_ptr<controller::Context>& aContext)
      : mContext(aContext)
  {
  }

  void EnvironmentController::provideObstacles(const Obstacles& aObstacles)
  {
    mContext->provideObstacles(aObstacles);
  }

  void EnvironmentController::executeHardstop(bool hardstop)
  {
    if (typeid(*mContext->currentState()) ==
            typeid(controller::EmergencyStop) ||
        hardstop == true)
      mContext->hardStop(hardstop);
  }

  void EnvironmentController::provideCup(const Cup& aCup)
  {
    mContext->foundCup(aCup);
    std::thread(&controller::Context::run, mContext).detach();
  }

} // namespace environment_controller