#include "sim_robot/Command.hpp"

#include "sim_robot/types.hpp"
#include <vector>

namespace data
{
  /**
   * @brief CommandData struct containing a cSpeedFactor_ and cTheta_
   */
  struct CommandData
  {

    std::vector<jointRad_t> cTheta_;
    jointVel_t cSpeedFactor_;
    /**
     * @brief Construct a new command Data struct
     * @param aTheta
     * @param aSpeedFactor
     *
     */
    CommandData(std::vector<jointRad_t> aTheta, jointVel_t aSpeedFactor);
    /**
     * @brief Destroy the CommandData struct
     *
     */
    ~CommandData() = default;

    /**
     * @brief Getter for mRad checks whether cSpeedFactor_ is  between 0.0 and
     * robotcontrollerplugin::cMaxSpeedfactor
     *
     * @return jointRad_t
     */

    std::vector<jointRad_t>& mTheta();

    /**
     * @brief Getter for mSpeedFactor checks whether mSpeedFactor is below 0
     *
     * @return jointVel_t
     */

    jointVel_t& mSpeedFactor();

    CommandData& operator=(const CommandData&) = default;
  };
} // namespace data
