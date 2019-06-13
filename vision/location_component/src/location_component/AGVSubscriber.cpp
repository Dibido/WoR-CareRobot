#include "location_component/AGVSubscriber.hpp"

namespace location_component
{
  AGVSubscriber::AGVSubscriber(const std::string& aTopicName,
                               const std::shared_ptr<DetectAGV>& aDetectAGV)
      : mSubscriber(
            mHandle.subscribe(aTopicName,
                              cQueue_size,
                              &location_component::AGVSubscriber::AGVCallBack,
                              this)),
        mDetectAGV(aDetectAGV)
  {
  }

  AGVSubscriber::~AGVSubscriber()
  {
    
  }

  void AGVSubscriber::publishAGVSpeed(const location_component::AGV& aAGV)
  {
    mDetectAGV->setAGVSpeed(aAGV);
  }

  void AGVSubscriber::AGVCallBack(const sensor_interfaces::AGVSpeedPtr& aSpeed)
  {
    location_component::AGV mAGV(aSpeed->speed);
    publishAGVSpeed(mAGV);
  }

} // namespace location_component