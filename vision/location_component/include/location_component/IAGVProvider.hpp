#ifndef IAGV_PROVIDER_HPP
#define IAGV_PROVIDER_HPP

#include <ros/ros.h>
#include "location_component/AGV.hpp"

namespace location_component
{
  class IAGVProvider
  {
      public:
    IAGVProvider(){};
    ~IAGVProvider() = default;

    virtual void publishAGVSpeed(const location_component::AGV& aAGV) = 0;
    
  };

} // namespace location_component

#endif