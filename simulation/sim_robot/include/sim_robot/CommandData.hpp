#include "sim_robot/Command.hpp"

#include "sim_robot/types.hpp"
#include <vector>

namespace data
{
  struct CommandData
  {
    CommandData(std::vector<jointRad_t> aRad, jointVel_t aSpeedFactor);
    ~CommandData() = default;

    /**
     * @brief Getter for mRad checks whether mRad is below 0
     *
     * @return jointRad_t
     */

    std::vector<jointRad_t>& mRad();

    /**
     * @brief Getter for mSpeedFactor checks whether mSpeedFactor is below 0
     *
     * @return jointVel_t
     */

    jointVel_t& mSpeedFactor();

      private:
    std::vector<jointRad_t> mRad_;
    jointVel_t mSpeedFactor_;
  };
} // namespace data