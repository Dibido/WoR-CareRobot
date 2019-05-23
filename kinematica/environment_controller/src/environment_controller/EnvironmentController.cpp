#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/SafetyController.hpp"
namespace environment_controller
{
  EnvironmentController::EnvironmentController()
  {
  }

  void EnvironmentController::provideObstacles(const Obstacles& aObstacles)
  {
    std::cout << "Added :" << aObstacles.size() << " obstacles!" << std::endl;
  }

  void EnvironmentController::executeHardstop(bool hardstop)
  {
    std::cout << "Hardstop: " << hardstop << std::endl;
  }

} // namespace environment_controller