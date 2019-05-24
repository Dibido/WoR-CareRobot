#ifndef ROSSERVICEOBSTACKLES_HPP
#define ROSSERVICEOBSTACKLES_HPP

#include "environment_controller/ICupProvider.hpp"
#include "kinematica_msgs/Cup.h"
#include <ros/ros.h>
#include "std_msgs/String.h"

namespace location_component
{

  class RosServiceCup : public environment_controller::ICupProvider
  {
      public:
    RosServiceCup(ros::NodeHandle& aNodeHandle);
    virtual ~RosServiceCup() = default;

    virtual void
        foundCup(const environment_controller::Cup& aCup);

      private:
    ros::NodeHandle& mNodeHandle;
    ros::Publisher mCupPublisher;
  };

} // namespace location_component

#endif // ROSSERVICEOBSTACKLES_HPP