
#ifndef KINECT_IMAGE_VISUALIZER_HPP
#define KINECT_IMAGE_VISUALIZER_HPP

#include <ros/ros.h>

#include "Visualizer.hpp"

// messages
#include <sensor_msgs/Image.h>

namespace visualization {
/**
 * Class to modify and publish kinect image data on callback
 */
class KinectImageVisualizer
    : public Visualizer<sensor_msgs::Image, sensor_msgs::Image> {
public:
  /**
   * @brief Create a new object
   * @author Bas Cop
   * @param frameId name of the message frames
   */
  explicit KinectImageVisualizer(const std::string &frameId);

  virtual ~KinectImageVisualizer() = default;

  /**
   * @brief Processes the incoming kinect image message
   * @author : Jelle Bouwhuis
   * @param inMesssage : Incoming kinect message
   */
  void callback(const sensor_msgs::Image &inMesssage) override;
};

} // namespace visualization

#endif // KINECT_IMAGE_VISUALIZER_HPP
