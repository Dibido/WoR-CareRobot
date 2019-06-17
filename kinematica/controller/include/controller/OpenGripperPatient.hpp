#ifndef OPEN_GRIPPER_PATIENT_HPP
#define OPEN_GRIPPER_PATIENT_HPP

// Local
#include "Context.hpp"
#include "OpenGripper.hpp"
namespace controller
{
  /**
   * @class OpenGripperPatient
   *
   * @brief OpenGripperPatient is the class which represents the
   * OpenGripperPatient state.
   *
   */
  class OpenGripperPatient : public OpenGripper
  {
      public:
    /**
     * @brief Construct a new OpenGripperPatient object
     *
     */
    OpenGripperPatient() = default;
    /**
     * @brief Destroy the OpenGripperPatient object
     *
     */
    virtual ~OpenGripperPatient() = default;

    virtual void transition(Context* aContext);

      private:
    ros::Time mReleaseTime;
  };
} // namespace controller
#endif // OPEN_GRIPPER_HPP