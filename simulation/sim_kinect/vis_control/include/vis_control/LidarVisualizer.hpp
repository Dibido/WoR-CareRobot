#ifndef LIDAR_VISUALIZER_HPP
#define LIDAR_VISUALIZER_HPP

#include "Visualizer.hpp"

// messages
#include <sensor_msgs/LaserScan.h>

namespace visualization {

/**
 * Class to modify and publish lidar data on callback
 */
class LidarVisualizer
    : public Visualizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan> {
public:
  /**
   * @brief Create a new object
   * @author Bas Cop
   * @param frameId name of the message frames
   */
  explicit LidarVisualizer(const std::string &frameId);

  virtual ~LidarVisualizer() = default;

  /**
   * @brief Processes the incoming lidar message
   * @author : Jelle Bouwhuis
   * @param inMessage : Incoming lidar message
   */
  void callback(const sensor_msgs::LaserScan &inMessage) override;
};

} // namespace visualization

#endif // LIDAR_VISUALIZER_HPP
