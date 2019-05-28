#ifndef AGV_SUBSCRIBER_HPP
#define AGV_SUBSCRIBER_HPP

#include "location_component/DetectAGV.hpp"
#include "location_component/IAGVProvider.hpp"
#include "ros/ros.h"
#include "sensor_interfaces/AGVSpeed.h"
#include "std_msgs/Float32.h"

namespace location_component
{
  class AGVSubscriber : public IAGVProvider
  {
      public:
    /**
     * @brief Construct a new AGVSubscriber object
     *
     * @param aTopicName - The topic name that is used to subscribe to receive
     * messages
     * @param aDetectAGV - A shared pointer of the AGV object. Used to set the
     * AGV speed when a update is received.
     */
    AGVSubscriber(const std::string& aTopicName,
                  const std::shared_ptr<DetectAGV>& aDetectAGV);

    /**
     * @brief Destroy the AGVSubscriber object
     *
     */
    virtual ~AGVSubscriber() = default;

    /**
     * @brief The inherited function from the IAgvProvided interface
     *
     * @param aAGV
     */
    virtual void publishAGVSpeed(const location_component::AGV& aAGV);
    void AGVCallBack(const sensor_interfaces::AGVSpeedPtr& aSpeed);

      private:
    /**
     * @brief Ros variable used to subscribe to the topic
     *
     */
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;

    /**
     * @brief Pointer to the DetectAGV object that is used to detect agv and the
     * cups
     *
     */
    std::shared_ptr<DetectAGV> mDetectAGV;

    /**
     * @brief The Queue size for the receiving messages from the AGV
     *
     */
    const unsigned cQueue_size = 1000;
  };

} // namespace location_component

#endif