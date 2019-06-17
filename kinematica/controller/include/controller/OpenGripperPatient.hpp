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
   * @brief OpenGripperPatient is the class which represents the OpenGripperPatient state.
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
    virtual ~OpenGripperPatient() = defaut;
    /**
     * @brief entryAction is being called when the OpenGripperPatient state is being
     * entered.
     *
     * @details The entryAction will calculate the time it takes to open the
     * gripper. It will furthermore start opening the gripper. After telling the
     * gripper to open the thread will be put to sleep for the time
     * it takes to open the gripper minus a waittime.
     *
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void entryAction(Context* aContext) override;

    /**
     * @brief doActivity is continiously being called while the system is in the
     * OpenGripperPatient.
     *
     * @details The doActivity function will check whether the time it
     * takes to open the gripper has passed. Once the time it took to open the
     * gripper it will transit to the Ready state.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void doActivity(Context* aContext) override;
    /**
     * @brief exitAction is being called when the OpenGripperPatient state is being
     * exited.
     *
     * @details At this moment the exitAction is not used.
     *
     * @param aContext is an object which gives the states an interface to the
     * "outside world".
     */
    void exitAction(Context* aContext) override;

    virtual void transition(Context* aContext) override;

      private:
    ros::Time mReleaseTime;
  };
} // namespace controller
#endif // OPEN_GRIPPER_HPP