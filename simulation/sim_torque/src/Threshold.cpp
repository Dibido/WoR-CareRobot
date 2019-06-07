#include "sim_torque/Threshold.hpp"

sim_torque::Threshold::Threshold(const double aLower, const double aUpper)
    : mLower(aLower), mUpper(aUpper)
{
}

bool sim_torque::Threshold::isWithin(const double aValue)
{
  return aValue >= mLower && aValue <= mUpper;
}