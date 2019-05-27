#ifndef AGV_SUBSCRIBER_HPP
#define AGV_SUBSCRIBER_HPP

#include "location_component/DetectAGV.hpp"
#include "location_component/IAgvProvider.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace location_component
{
  class AGVSubscriber : public IAgvProvider
  {
      public:
    AGVSubscriber(const std::string& aTopicName,
                  const std::shared_ptr<DetectAGV>& aDetectAGV);

    virtual ~AGVSubscriber() = default;

    virtual void publishAGVSpeed(const location_component::AGV& aAGV);
    void AGVCallBack(const std_msgs::Float64& aSpeed);

      private:
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    std::shared_ptr<DetectAGV> mDetectAGV;

    const unsigned cQueue_size = 1000;
  };

} // namespace location_component

#endif