#ifndef IAGV_PROVIDER_HPP
#define IAGV_PROVIDER_HPP

#include <ros/ros.h>
#include "location_component/AGV.hpp"

namespace location_component
{
  class IAgvProvider
  {
      public:
    IAgvProvider(/* args */);
    ~IAgvProvider() = default;

    virtual void publishAGVSpeed(const location_component::AGV& aAGV) = 0;

      private:
    ros::NodeHandle& mNodeHandle;
    ros::Publisher mCupPublisher;
  };

} // namespace location_component

#endif