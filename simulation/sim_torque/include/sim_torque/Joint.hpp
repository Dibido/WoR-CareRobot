#ifndef JOINT_HPP
#define JOINT_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "Threshold.hpp"

namespace sim_torque
{
  class Joint
  {
      private:
    void callback(const ConstWrenchStampedPtr& aMsg);
    double calculateLength(double aX, double aY, double aZ);
    double pythagoras(double aA, double aB);

    /* data */
    gazebo::transport::NodePtr mNode;
    gazebo::transport::SubscriberPtr mSubscriber;
    double mForce;
    double mTorque;

    Threshold mForceThresholdAcceleration;
    Threshold mForceThresholdNominal;
    Threshold mTorqueThresholdAcceleration;
    Threshold mTorqueThresholdNominal;

      public:
    Joint(unsigned short aNumber);
    ~Joint() = default;

    bool isWithinThresholdsNominal();
    bool isWithinThresholdsAcceleration();
  };
} // namespace sim_torque

#endif