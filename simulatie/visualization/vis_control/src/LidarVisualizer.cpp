#include <vis_control/LidarVisualizer.hpp>

namespace visualization {

LidarVisualizer::LidarVisualizer(const std::string &frameId)
    : Visualizer(frameId) {}

void LidarVisualizer::callback(const sensor_msgs::LaserScan &inMessage) {
  // this message does not need conversion so it can be published right away
  publishWithHeader(inMessage);
}

} // namespace visualization
