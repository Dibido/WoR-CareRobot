#include "sim_robot/Command.hpp"

#include "sim_robot/types.hpp"
#include <vector>

namespace control_data
{
  /**
   * @brief CommandData struct containing a mSpeedFactor_ and mTheta_
   */
  struct CommandData
  {

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
     * throw std::invalid_argument if not between cMinRad and
     * cMaxRad
     *
     * @return jointRad_t
     */

    std::vector<jointRad_t>& getTheta();

    /**
     * @brief Getter for mSpeedFactor checks whether mSpeedFactor is between 0.0
     * and cMaxSpeedfactor
     * throw std::invalid_argument if not between 0.0
     * and cMaxSpeedfactor
     *
     * @return jointVel_t
     */

    jointVel_t& getSpeedFactor();

    CommandData& operator=(const CommandData&) = default;

      private:
    std::vector<jointRad_t> mTheta_;
    jointVel_t mSpeedFactor_;
  };
} // namespace control_data
