
#ifndef PROJECT_JOINTINFO_HPP
#define PROJECT_JOINTINFO_HPP

#include "types.hpp"
#include <gazebo/physics/Joint.hh>

namespace gazebo
{
  /**
   * @brief Simulation of a robot joint, has all the information like min / max
   * pwm and radians. Controls the Gazebo joint.
   */
  class JointController
  {
      public:
    /**
     * @brief Construct a joint controller
     * @param joint : joint pointer to the gazebo physical joint
     * @param mName : Name of the joint
     * @param mChannel: Index / Channel of the joint
     * @param mMinPw : Minimum pulse width
     * @param mMaxPw : Maximum pulse width
     * @param mMinRad : Minimum radians (to move joint to)
     * @param mMaxRad : Maximum radians (to move joint to)
     * @param  : Maximum velocity in degrees per second (ex. 180 will
     * result in 1 second per 180 degrees)
     */
    JointController(physics::JointPtr& joint,
                    const std::string& mName,
                    jointChannel_t mChannel,
                    jointPw_t mMinPw,
                    jointPw_t mMaxPw,
                    jointRad_t mMinRad,
                    jointRad_t mMaxRad,
                    jointVel_t mMaxVel);

    JointController(const JointController& other);
    JointController& operator=(const JointController& other);
    bool operator==(const JointController& other) const;
    bool operator!=(const JointController& other) const;

    /**
     * @brief Run the joint once and update the gazebo model joint
     * Called by update from robot controller
     */
    void update();

    /**
     * @brief Check if given pulse width is in valid range for this joint
     * @param mPw : pulse width to check if it is in range
     * @return true if in range
     */
    bool inRange(jointPw_t aPw) const;

    /**
     * Move the joint to given position with given speed, only if given pulse
     * width is in range
     * @param mPw : pulse width to move to
     * @param mSpeed : move with this speed (if faster than given time)
     * @param time : move in this time (if faster than speed)
     * @return true if move was successful
     */
    bool move(jointPw_t aPw,
              jointVel_t aSpeed,
              commandTime_t aTime,
              double aUpdateRate);

    bool moveTheta(jointRad_t aRad,
                   jointVel_t aSpeedFactor,
                   commandTime_t aTime,
                   double aUpdateRate);
    /**
     * @brief Stop the joint movement
     */
    void stop();

    /**
     * @brief converts a scale to the range of radials of this joint
     *
     * @param aScale the value that needs to be conversed
     * @param aMinScale the minumum of the value range
     * @param aMaxScale the maximum of the value range
     * @return jointRad_t the converted value in radials.
     */
    jointRad_t converseScaleToRad(double aScale,
                                  double aMinScale = 0,
                                  double aMaxScale = 1);

    jointRad_t getTargetPos() const;
    jointRad_t getCurrentPos() const;
    jointVel_t getCurrentVel() const;
    jointVel_t getMaxVel() const;
    double getCurrentForce() const;

    /**
     * @brief Hard set current position, no checks.
     * @param aCurrentPos.
     */
    void setCurrentPos(jointRad_t aCurrentPos);
    /**
     * @brief Hard set current velocity, no checks.
     * @param aCurrentVel.
     */
    void setCurrentVel(jointVel_t aCurrentVel);

      private:
    /**
     * @brief convert PW to radians.
     * @param aPw argument to convert.
     */
    double convertPw2Radians(jointPw_t aPw) const;

    /**
     * @brief convert degrees to radians.
     * @param aPw argument to convert.
     */
    jointRad_t convertDegrees2Radians(jointVel_t mVel) const;
    /**
     *@brief Step once towards target position with step size
     */
    void run();

    // Simulation joint
    physics::JointPtr joint;

    // Settings
    std::string mName;
    jointChannel_t mChannel;
    jointPw_t mMinPw;
    jointPw_t mMaxPw;
    jointRad_t mMinRad;
    jointRad_t mMaxRad;
    jointVel_t mMaxVel;

    // Current values
    jointRad_t mCurrentPos;
    jointVel_t mCurrentVel;
    double mCurrentForce;

    // For movement simulation
    jointRad_t mTargetPos;
    jointRad_t mStepSize;
  };
  bool equalsDouble(const double& a, const double& b);
} // namespace gazebo

#endif // PROJECT_JOINTINFO_HPP
