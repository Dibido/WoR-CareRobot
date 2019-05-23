#ifndef ROSSERVICEOBSTACKLES_HPP
#define ROSSERVICEOBSTACKLES_HPP

#include "environment_controller/IObstacles.hpp"
#include <ros/ros.h>

namespace location_component
{

  class RosServiceObstacles : public environment_controller::IObstacles
  {
      public:
    RosServiceObstacles(ros::NodeHandle& aNodeHandle);
    virtual ~RosServiceObstacles() = default;

    virtual void parseObstacles(const environment_controller::Obstacles& aObstacles);

      private:
    ros::NodeHandle& mNodeHandle;
    ros::ServiceServer mService;
  };

} // namespace location_component


#endif //ROSSERVICEOBSTACKLES_HPP