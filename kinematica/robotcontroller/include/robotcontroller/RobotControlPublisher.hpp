#ifndef ROBOTCONTROL_PUBLISHER_HPP
#define ROBOTCONTROL_PUBLISHER_HPP

#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include "robotcontroller_msgs/Control.h"   
#include "ros/ros.h"

namespace robotcontroller {

class RobotControlPublisher
{
    public:
    explicit RobotControlPublisher(ros::NodeHandle& lN);
    void publish(const double lSf, const std::vector<kinematics::Link>& joints);

    private:
    ros::NodeHandle& mN;
};
}

#endif //ROBOTCONTROL_PUBLISHER_HPP