#include "location_component/AGVSubscriber.hpp"

namespace location_component
{
  AGVSubscriber::AGVSubscriber(const std::string& aTopicName,
                               const std::shared_ptr<DetectAGV>& aDetectAGV) : 
            mSubscriber(mHandle.subscribe(
            aTopicName,
            cQueue_size,
            &location_component::AGVSubscriber::AGVCallBack,
            this)), mDetectAGV(aDetectAGV)
  {
  }

  void AGVSubscriber::publishAGVSpeed(const location_component::AGV& aAGV)
  {
      mDetectAGV->setAGVSpeed(aAGV);
  }

  void AGVSubscriber::AGVCallBack(const std_msgs::Float32& aSpeed)
  {
    location_component::AGV mAGV(aSpeed.data);
    publishAGVSpeed(mAGV);
  }

} // namespace location_component