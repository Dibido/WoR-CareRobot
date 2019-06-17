#ifndef OPEN_GRIPPER_TABLE_HPP
#define OPEN_GRIPPER_TABLE_HPP

// Local
#include "Context.hpp"
#include "OpenGripper.hpp"
namespace controller
{
  /**
   * @class OpenGripperTable
   *
   * @brief OpenGripperTable is the class which represents the OpenGripperTable state.
   *
   */
  class OpenGripperTable : public OpenGripper
  {
      public:
    /**
     * @brief Construct a new OpenGripperTable object
     *
     */
    OpenGripperTable() = default;
    /**
     * @brief Destroy the OpenGripperTable object
     *
     */
    virtual ~OpenGripperTable() = defaut;
    /**
     * @brief entryAction is being called when the OpenGripperTable state is being
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
     * OpenGripperTable.
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
     * @brief exitAction is being called when the OpenGripperTable state is being
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
#endif // OPEN_GRIPPER_TABLE_HPP