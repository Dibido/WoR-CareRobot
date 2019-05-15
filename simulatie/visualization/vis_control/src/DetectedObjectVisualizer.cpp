#include <vis_control/DetectedObjectVisualizer.hpp>

namespace visualization {
DetectedObjectVisualizer::DetectedObjectVisualizer(const std::string &frameId)
    : Visualizer(frameId), previousAmountOfDetectedObjects(0) {}

void DetectedObjectVisualizer::callback(
    const vision::DetectedObjects &incommingMessage) {
  auto objects = getObjects(incommingMessage);

  if (OneOrMoreObjectsDisappeared(objects.size())) {
    publishRemoveAllMarkers();
  }

  visualization_msgs::MarkerArray outgoingMessage =
      getMarkerArrayMessage(objects);

  this->previousAmountOfDetectedObjects = objects.size();
  publish(outgoingMessage);
}

std::vector<DetectedObject> DetectedObjectVisualizer::getObjects(
    const vision::DetectedObjects &incommingMessage) const {
  std::vector<DetectedObject> objects;

  size_t amountOfDetectedObjects = incommingMessage.position.size();

  for (size_t objectIndex = 0; objectIndex < amountOfDetectedObjects;
       ++objectIndex) {
    objects.push_back(getObject(incommingMessage, objectIndex));
  }

  return objects;
}

DetectedObject DetectedObjectVisualizer::getObject(
    const vision::DetectedObjects &incommingMessage, size_t index) const {
  double width = incommingMessage.width.at(index);
  double height = incommingMessage.height.at(index);
  double depth = incommingMessage.depth.at(index);

  double x = incommingMessage.position.at(index).x + (width / 2);
  double y = incommingMessage.position.at(index).y + (height / 2);
  double z = incommingMessage.position.at(index).z - (depth / 2);

  return DetectedObject(index, x, y, z, width, height, depth);
}

visualization_msgs::MarkerArray DetectedObjectVisualizer::getMarkerArrayMessage(
    std::vector<DetectedObject> &objects) const {
  visualization_msgs::MarkerArray markerArrayMessage;

  for (DetectedObject object : objects) {
    markerArrayMessage.markers.push_back(getMarkerMessage(object));
  }

  return markerArrayMessage;
}

visualization_msgs::Marker
DetectedObjectVisualizer::getMarkerMessage(const DetectedObject &object) const {
  visualization_msgs::Marker markerMessage;

  double width = object.getWidth();
  double height = object.getHeight();

  markerMessage.header.frame_id = frameId;
  markerMessage.header.stamp = ros::Time::now();

  markerMessage.ns = "Object";
  markerMessage.id = static_cast<int>(object.getIndex());

  markerMessage.type = visualization_msgs::Marker::CUBE;
  markerMessage.action = visualization_msgs::Marker::ADD;

  markerMessage.pose.position.x = object.getX();
  markerMessage.pose.position.y = object.getY();
  markerMessage.pose.position.z = (object.getZ() - 1) / 2;

  markerMessage.pose.orientation.x = static_cast<double>(0.0);
  markerMessage.pose.orientation.y = static_cast<double>(0.0);
  markerMessage.pose.orientation.z = static_cast<double>(0.0);
  markerMessage.pose.orientation.w = static_cast<double>(1.0);

  markerMessage.scale.x = width;
  markerMessage.scale.y = height;
  markerMessage.scale.z = object.getZ() + 1;

  markerMessage.color.r = static_cast<float>(0.0);
  markerMessage.color.g = static_cast<float>(1.0);
  markerMessage.color.b = static_cast<float>(0.0);
  markerMessage.color.a = static_cast<float>(1.0);

  markerMessage.lifetime = ros::Duration();

  return markerMessage;
}

bool DetectedObjectVisualizer::OneOrMoreObjectsDisappeared(
    size_t amountOfMarkers) const {
  return previousAmountOfDetectedObjects > amountOfMarkers;
}

void DetectedObjectVisualizer::publishRemoveAllMarkers() const {
  visualization_msgs::MarkerArray clearMessage;
  visualization_msgs::Marker removeMarker;

  removeMarker.header.frame_id = frameId;
  removeMarker.header.stamp = ros::Time::now();
  removeMarker.action = visualization_msgs::Marker::DELETEALL;

  clearMessage.markers.push_back(removeMarker);

  publish(clearMessage);
}

} // namespace visualization
