#include "sim_robot/CommandData.hpp"

namespace data
{

  data::CommandData::CommandData(std::vector<jointRad_t> aRad,
                                 jointVel_t aSpeedFactor)
      : mRad_(aRad), mSpeedFactor_(aSpeedFactor)
  {
  }

  std::vector<jointRad_t>& data::CommandData::mRad()
  {
    for (const auto& r : mRad_)
    {

      if (r < 0)
      {
        throw std::invalid_argument("mRad_ is smaller than 0: " +
                                    std::to_string(r));
      }
    }
    return mRad_;
  }
  jointVel_t& data::CommandData::mSpeedFactor()
  {
    if (mSpeedFactor_ < 0)
    {
      throw std::invalid_argument("B is smaller than 0: " +
                                  std::to_string(mSpeedFactor_));
    }
    return mSpeedFactor_;
  }

} // namespace data
