#include "sim_torque/Joint.hpp"

sim_torque::Joint::Joint(unsigned short aNumber)
{
  mNode = boost::make_shared<Node>(new gazebo::transport::Node());
  mNode->Init();
  std::string lTopicToSub =
      "/gazebo/default/Franka_panda/_joint" + aNumber + "/force_torque/wrench";
  gazebo::transport::SubscriberPtr mSubscriber =
      mNode->Subscribe(lTopicToSub, callback);
}

void sim_torque::Joint::callback(const ConstWrenchStampedPtr& aMsg)
{
  mForce =
      calculateLength(_msg->wrench().force().x(), _msg->wrench().force().y(),
                      _msg->wrench().force().z());
  mTorque =
      calculateLength(_msg->wrench().torque().x(), _msg->wrench().torque().y(),
                      _msg->wrench().torque().z());
}

double sim_torque::Joint::calculateLength(double aX, double aY, double aZ);
{
  return pythagoras(pythagoras(x, y), z);
}

double sim_torque::Joint::pythagoras(double aA, double aB)
{
  return sqrt(pow(a, 2.0) + pow(b, 2.0));
}

bool sim_torque::Joint::isWithinThresholdsNominal()
{
  return mForceThresholdNominal.isWithin(mForce) &&
         mTorqueThresholdNominal.isWithin(mTorque);
}

bool sim_torque::Joint::isWithinThresholdsAcceleration()
{
  return mForceThresholdAcceleration.isWithin(mForce) &&
         mTorqueThresholdAcceleration.isWithin(mTorque);
}