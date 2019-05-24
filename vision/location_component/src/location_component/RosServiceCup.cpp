#include "location_component/RosServiceCup.hpp"

namespace location_component
{

  RosServiceCup::RosServiceCup(ros::NodeHandle& aNodeHandle) : mNodeHandle(aNodeHandle)
  {
    mCupPublisher = aNodeHandle.advertise<kinematica_msgs::Cup>(environment_controller::cCupTopicName, 1000);
  }


  void RosServiceCup::foundCup(const environment_controller::Cup& aCup)
  {


  }
} // namespace location_component