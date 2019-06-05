#include <vis_control/KinectPointsVisualizer.hpp>

namespace visualization {

KinectPointsVisualizer::KinectPointsVisualizer(const std::string &frameId)
    : Visualizer(frameId) {}
void KinectPointsVisualizer::callback(
    const sensor_msgs::PointCloud2 &inMessage) {
  // this message does not need conversion so it can be published right away
  publishWithHeader(inMessage);
}

} // namespace visualization
