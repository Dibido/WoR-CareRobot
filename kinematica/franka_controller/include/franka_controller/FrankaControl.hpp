#ifndef FRANKA_CONTROL_HPP
#define FRANKA_CONTROL_HPP

#include "FrankaConsts.hpp"
#include "FrankaFeedback.hpp"
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

#include <memory>
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
     * @brief move the robotarm in position
     *
     * @param aConfig the config where it is set tp
     * @param aSpeedFactor the speedfactor
     */
    void executeMovement(std::array<double, cDegreesOfFreedom>& aConfig,
                         double aSpeedFactor);

    /**
     * @brief open the gripper to a certain meters
     *
     * @param aWidth the width of the gripper to move to
     * @param aSpeedFactor the speed to open or close the gripper
     */
    void moveGripper(double aWidth, double aSpeedFactor);

    /**
     * @brief stop the robotarm
     *
     * @param aStop if the robotarm needs to stop
     */
    void stopRobot(bool aStop);

      private:
    franka::Robot mRobot;
    franka::Gripper mGripper;
    bool mStop;
    FrankaFeedback mFeedback;
  };
} // namespace franka_controller
#endif // FRANKA_CONTROL_HPP