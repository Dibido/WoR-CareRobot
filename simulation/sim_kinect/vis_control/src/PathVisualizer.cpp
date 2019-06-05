#include <vis_control/PathVisualizer.hpp>

namespace visualization {
PathVisualizer::PathVisualizer(const std::string &frameId)
    : Visualizer(frameId), path() {}

void PathVisualizer::callback(const path_planner::PathCoordinate &inMessage) {
  nav_msgs::Path outMessage;
  geometry_msgs::PoseStamped position;

  position.pose.position.x = millimetreToDecimetre(inMessage.coordinate.x);
  position.pose.position.y = millimetreToDecimetre(inMessage.coordinate.y);
  position.pose.position.z = millimetreToDecimetre(inMessage.coordinate.z);

  position.pose.orientation.x = static_cast<double>(0.0);
  position.pose.orientation.y = static_cast<double>(0.0);
  position.pose.orientation.z = static_cast<double>(0.0);
  position.pose.orientation.w = static_cast<double>(1.0);

  path.push_back(position);

  outMessage.poses = path;
  publishWithHeader(outMessage);
}

double PathVisualizer::millimetreToDecimetre(double millimetres) const {
  return millimetres / static_cast<double>(100);
}

} // namespace visualization
