#ifndef GOAL_PUBLISHER_HPP
#define GOAL_PUBLISHER_HPP

#include "environment_controller/EnvironmentConsts.hpp"
#include "environment_controller/IGoalProvider.hpp"
#include "environment_controller/Position.hpp"
#include "geometry_msgs/Point.h"
#include "kinematica_msgs/Goal.h"
#include "ros/ros.h"

namespace userinterface
{

  /**
   * @brief GoalPublisher class inherited from IReleaseTimeProvider.
   *
   * See
   * https://git.icaprojecten.nl/stash/projects/EBGURG/repos/wor-18-19-s2/browse/kinematica/environment_controller/include/environment_controller/IGoalProvider.hpp
   * for more information.
   *
   * @pre The desired x,y,z coordinates of where the cup needs to be placed must
   * be known
   * @post A ROS message with the coordinates will be sent to the provided
   * topic.
   */
  class GoalPublisher : public environment_controller::IGoalProvider
  {
      public:
    /**
     * @brief Constructor
     */
    GoalPublisher();

    /**
     * @brief Destructor
     */
    virtual ~GoalPublisher();

    /**
     * @brief Inherited method from base class. A ROS message with aPosition
     * will be sent to the given topic.
     * @param aPosition
     */
    virtual void selectGoalPosition(
        const environment_controller::Position& aPosition) override;

    /**
     * @brief State boolean to store if the msg has been sent.
     */
    bool mMsgSent = false;

      private:
    ros::NodeHandle goalPublisherNodeHandle;
    ros::Publisher chatter_pub;
  };

} // namespace userinterface

#endif // GOAL_PUBLISHER_HPP
