#ifndef DETECTED_OBJECT_VISUALIZER_HPP
#define DETECTED_OBJECT_VISUALIZER_HPP

#include <ros/ros.h>
#include <stddef.h>
#include <vector>

#include "DetectedObject.hpp"
#include "Visualizer.hpp"

// message(s)
#include <vision/DetectedObjects.h>         // in message
#include <visualization_msgs/MarkerArray.h> // out message

namespace visualization {

/**
 * Class to modify and publish vision object data on callback
 */
class DetectedObjectVisualizer
    : public Visualizer<vision::DetectedObjects,
                        visualization_msgs::MarkerArray> {
public:
  /**
   * @brief Create a new object
   * @author : Wouter van Uum
   * @param frameId : name of the message frames
   */
  explicit DetectedObjectVisualizer(const std::string &frameId);

  virtual ~DetectedObjectVisualizer() = default;

  /**
   * @brief Processes the incoming detected objects message
   * @author : Wouter van Uum
   * @param incommingMessage : Incoming detected objects message
   */
  void callback(const vision::DetectedObjects &incommingMessage) override;

private:
  std::vector<DetectedObject>
  getObjects(const vision::DetectedObjects &incommingMessage) const;

  DetectedObject getObject(const vision::DetectedObjects &incommingMessage,
                           size_t index) const;

  visualization_msgs::MarkerArray
  getMarkerArrayMessage(std::vector<DetectedObject> &objects) const;

  visualization_msgs::Marker
  getMarkerMessage(const DetectedObject &object) const;

  bool OneOrMoreObjectsDisappeared(size_t amountOfMarkers) const;

  void publishRemoveAllMarkers() const;

  size_t previousAmountOfDetectedObjects;
};

} // namespace visualization

#endif // DETECTED_OBJECT_VISUALIZER_HPP
