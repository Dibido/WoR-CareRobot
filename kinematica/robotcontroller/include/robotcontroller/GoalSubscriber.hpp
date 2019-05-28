#ifndef GOALSUBSCRIBER_PROVIDER_HPP
#define GOALSUBSCRIBER_PROVIDER_HPP

namespace environment_controller
{

  /**
   * @brief
   *
   */
  class GoalSubscriber
  {

      public:
    GoalSubscriber(const std::string& aTopicName);

    virtual ~GoalSubscriber() = default;

      private:
  };

} // namespace environment_controller

#endif // GOALSUBSCRIBER_PROVIDER_HPP