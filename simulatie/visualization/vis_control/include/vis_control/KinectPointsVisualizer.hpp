
#ifndef KINECT_POINTS_VISUALIZER_HPP
#define KINECT_POINTS_VISUALIZER_HPP

#include "Visualizer.hpp"

// message(s)
#include <sensor_msgs/PointCloud2.h>

namespace visualization {

/**
 * Class to modify and publish kinect pointcloud data on callback
 */
class KinectPointsVisualizer
    : public Visualizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> {
public:
  /**
   * @brief Create a new object
   * @author Bas Cop
   * @param frameId name of the message frames
   */
  explicit KinectPointsVisualizer(const std::string &frameId);

  virtual ~KinectPointsVisualizer() = default;

  /**
   * @brief Processes the incoming kinect pointcloud message
   * @author : Jelle Bouwhuis
   * @param inMessage : Incoming kinect message
   */
  void callback(const sensor_msgs::PointCloud2 &inMessage) override;
};

} // namespace visualization

#endif // KINECT_POINTS_VISUALIZER_HPP
