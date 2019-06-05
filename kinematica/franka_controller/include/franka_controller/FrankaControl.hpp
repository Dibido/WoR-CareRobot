#ifndef FRANKA_CONTROL_HPP
#define FRANKA_CONTROL_HPP
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

#include <string>

namespace franka_controller
{

  class FrankaControl
  {

      public:
    /**
     * @brief Construct a new Franka Control object
     *
     * @param anIp
     */
    FrankaControl(const std::string& anIp);
    virtual ~FrankaControl() = default;
    /**
     * @brief
     *
     * @param aConfig
     * @param aSpeedFactor
     */
    void executeMovement(std::array<double, 7>& aConfig, double aSpeedFactor);

      private:
    franka::Robot mRobot;
  };
} // namespace franka_controller
#endif // FRANKA_CONTROL_HPP