#include "location_component/AGV.hpp"

namespace location_component
{
  AGV::AGV(double aSpeed) : mSpeed(aSpeed)
  {
    if (aSpeed < 0)
    {
      throw std::range_error("AGV speed cannot be lower than zero");
    }
  }

  double& AGV::speed()
  {
    if (mSpeed < 0)
    {
      throw std::range_error("AGV speed cannot be lower than zero");
    }
    return mSpeed;
  }

  const double& AGV::speed() const
  {
    if (mSpeed < 0)
    {
      throw std::range_error("AGV speed cannot be lower than zero");
    }
    return mSpeed;
  }

} // namespace location_component