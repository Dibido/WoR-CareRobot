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
    virtual ~OpenGripperTable() = default;

    virtual void transition(Context* aContext);

      private:
    ros::Time mReleaseTime;
  };
} // namespace controller
#endif // OPEN_GRIPPER_TABLE_HPP