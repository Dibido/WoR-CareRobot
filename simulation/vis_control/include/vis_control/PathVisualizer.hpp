#ifndef PATH_VISUALIZER_HPP
#define PATH_VISUALIZER_HPP

#include <ros/ros.h>

#include "Visualizer.hpp"

// message(s)
#include <nav_msgs/Path.h>
#include <path_planner/PathCoordinate.h>

namespace visualization {

/**
 * Class to modify and publish path data on callback
 */
class PathVisualizer
    : public Visualizer<path_planner::PathCoordinate, nav_msgs::Path> {
public:
  /**
   * @brief Create a new object
   * @author : Wouter van Uum
   * @param frameId : name of the message frames
   */
  explicit PathVisualizer(const std::string &frameId);

  virtual ~PathVisualizer() = default;

  /**
   * @brief Processes the incoming path message
   * @author : Wouter van Uum
   * @param inMessage : Incoming path message
   */
  void callback(const path_planner::PathCoordinate &inMessage) override;

private:
  /**
   * @author Bas Cop
   * @brief Convert millimetres to decimetres
   * @param millimetres number that represents millimetres
   * @return number that representts decimetres
   */
  double millimetreToDecimetre(double millimetres) const;

  std::vector<geometry_msgs::PoseStamped> path; // all received points
};

} // namespace visualization

#endif // PATH_VISUALIZER_HPP
