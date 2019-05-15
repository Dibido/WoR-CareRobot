
#ifndef PROJECT_JOINTINFO_HPP
#define PROJECT_JOINTINFO_HPP

#include <gazebo/physics/Joint.hh>
#include "types.hpp"

namespace gazebo
{
/**
 * Simulation of a robot joint, has all the information like min / max pwm and radians. Controls the Gazebo joint
 */
class JointController
{
public:
  /**
   * Construct a joint controller
   * @param joint : joint pointer to the gazebo physical joint
   * @param name : Name of the joint
   * @param channel : Index / Channel of the joint
   * @param min_pw : Minimum pulse width
   * @param max_pw : Maximum pulse width
   * @param min_rad : Minimum radians (to move joint to)
   * @param max_rad : Maximum radians (to move joint to)
   * @param max_vel : Maximum velocity in radians per second (ex. M_PI_2 will result in 1 second per 90 degrees)
   */
  JointController(physics::JointPtr& joint, const std::string& name, jointChannel_t channel, jointPw_t min_pw,
                  jointPw_t max_pw, jointRad_t min_rad, jointRad_t max_rad, jointVel_t max_vel);

  JointController(const JointController& other);
  JointController& operator=(const JointController& other);
  bool operator==(const JointController& other) const;
  bool operator!=(const JointController& other) const;

  /**
   * Run the joint once and update the gazebo model joint
   * Called by update from robot controller
   */
  void update();

  /**
   * Check if given pulse width is in valid range for this joint
   * @param pw : pulse width to check if it is in range
   * @return true if in range
   */
  bool inRange(jointPw_t pw) const;

  /**
   * Move the joint to given position with given speed, only if given pulse width is in range
   * @param pw : pulse width to move to
   * @param speed : move with this speed (if faster than given time)
   * @param time : move in this time (if faster than speed)
   * @return true if move was successful
   */
  bool move(jointPw_t pw, jointVel_t speed, commandTime_t time, double updateRate);

  /**
   * Stop the joint movement
   */
  void stop();

  jointRad_t getTargetPos() const;
  jointRad_t getCurrentPos() const;
  jointVel_t getCurrentVel() const;

  /**
   * Hard set current position, no checks.
   * @param aCurrentPos
   */
  void setCurrentPos(jointRad_t aCurrentPos);

private:
  double convertPw2Radians(jointPw_t pw) const;

  /**
   * Step once towards target position with step size
   */
  void run();

  // Simulation joint
  physics::JointPtr joint;

  // Settings
  std::string name;
  jointChannel_t channel;
  jointPw_t min_pw;
  jointPw_t max_pw;
  jointRad_t min_rad;
  jointRad_t max_rad;
  jointVel_t max_vel;

  // Current values
  jointRad_t current_pos;
  jointVel_t current_vel;

  // For movement simulation
  jointRad_t target_pos;
  jointRad_t step_size;
};

}  // namespace gazebo

#endif  // PROJECT_JOINTINFO_HPP
