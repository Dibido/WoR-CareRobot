#ifndef CUP_SUBSCRIBER_HPP
#define CUP_SUBSCRIBER_HPP

#include "EnvironmentController.hpp"
#include "ICupProvider.hpp"
#include "environment_controller/Cup.hpp"
#include "kinematica_msgs/Cup.h"
#include "ros/ros.h"

namespace environment_controller
{
  /**
   * @brief
   *
   */
  class CupSubscriber : public ICupProvider
  {
      public:
    /**
     * @brief Construct a new Cup Subscriber object
     *
     * @param aTopicName
     */
    CupSubscriber(const std::string& aTopicName,
                  const std::shared_ptr<EnvironmentController>& aController);

    /**
     * @brief Destroy the Cup Subscriber object
     *
     */
    virtual ~CupSubscriber() = default;

    /**
     * @brief
     *
     * @param aMsg
     */
    void cupCallback(const kinematica_msgs::CupConstPtr& aMsg);

    /**
     * @see ICupProvider.hpp
     */
    virtual void foundCup(const Cup& aCup);

      private:
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    std::shared_ptr<EnvironmentController> mEnvironmentController;
  };

} // namespace environment_controller

#endif // CUP_SUBSCRIBER_HPP