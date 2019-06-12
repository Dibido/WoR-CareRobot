#include "sim_robot/Command.hpp"

#include "sim_robot/types.hpp"
#include <vector>

namespace stop_data
{
  /**
   * @brief StopData struct containing a cStop
   */
  struct StopData
  {

    jointStop_t mStop_ = 0;
    /**
     * @brief Construct a new  StopData struct
     * @param aStop
     *
     */
    StopData(jointStop_t aStop);
    /**
     * @brief Destroy the StopData struct
     *
     */
    ~StopData() = default;

    /**
     * @brief Getter for mStop_, checks whether mStop_ is  either true or
     * false
     *
     * @return jointStop_t
     */

    jointStop_t& isStopped();

    StopData& operator=(const StopData&) = default;
  };
} // namespace stop_data
