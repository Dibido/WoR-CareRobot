#include "sim_robot/Command.hpp"

#include "sim_robot/types.hpp"
#include <vector>

namespace control_data
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
     * @brief Getter for mTheta_ checks whether mTheta_ is between cMinRad and
     * cMaxRad
     *
     * @return jointRad_t
     */

    std::vector<jointRad_t>& mTheta();

    /**
     * @brief Getter for mSpeedFactor checks whether mSpeedFactor is between 0.0
     * and cMaxSpeedfactor
     *
     * @return jointVel_t
     */

    jointVel_t& mSpeedFactor();

    CommandData& operator=(const CommandData&) = default;
  };
} // namespace control_data
