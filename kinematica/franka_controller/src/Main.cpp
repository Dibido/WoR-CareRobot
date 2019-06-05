#include <ros/ros.h>

#include "controller/ControllerConsts.hpp"
#include "franka_controller/FrankaConsts.hpp"
#include "franka_controller/FrankaControl.hpp"
#include "franka_controller/FrankaMovementSubscriber.hpp"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "franka_controller");
  std::shared_ptr<franka_controller::FrankaControl> lFC =
      std::make_shared<franka_controller::FrankaControl>(
          franka_controller::cFrankaIp);
  franka_controller::FrankaMovementSubscriber lMoveSub(
      controller::cRobotCommandTopicName, lFC);
  ros::spin();
  return 0;
}
