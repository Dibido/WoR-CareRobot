#include <vis_control/KinectImageVisualizer.hpp>

namespace visualization {

KinectImageVisualizer::KinectImageVisualizer(const std::string &frameId)
    : Visualizer(frameId) {}
void KinectImageVisualizer::callback(const sensor_msgs::Image &inMesssage) {
  // this message does not need conversion so it can be published right away
  publishWithHeader(inMesssage);
}

} // namespace visualization
