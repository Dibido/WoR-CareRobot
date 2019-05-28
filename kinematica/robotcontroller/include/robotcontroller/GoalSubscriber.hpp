#ifndef GOALSUBSCRIBER_PROVIDER_HPP
#define GOALSUBSCRIBER_PROVIDER_HPP

#include "robotcontroller/IGoalProvider.hpp"

namespace robotcontroller
{

  /**
   * @brief
   *
   */
  class GoalSubscriber : public IGoalProvider
  {

      public:
    GoalSubscriber(const std::string& aTopicName);

    virtual ~GoalSubscriber() = default;

    virtual void selectGoalPosition(const Position& aPosition);

      private:
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
  };

} // namespace robotcontroller

#endif // GOALSUBSCRIBER_PROVIDER_HPP