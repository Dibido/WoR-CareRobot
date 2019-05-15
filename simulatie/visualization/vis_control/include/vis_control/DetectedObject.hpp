
#ifndef DETECTED_OBJECT_HPP
#define DETECTED_OBJECT_HPP

#include <stddef.h>

namespace visualization {

class DetectedObject {
public:
  DetectedObject(size_t id, double x, double y, double z, double width,
                 double height, double depth);

  virtual ~DetectedObject() = default;

  size_t getIndex() const;

  double getX() const;

  double getY() const;

  double getZ() const;

  double getWidth() const;

  double getHeight() const;

  double getDepth() const;

private:
  size_t index;
  double x; // x position of the center
  double y; // y position of the center
  double z; // z position of the center
  double width;
  double height;
  double depth;
};

} // namespace visualization

#endif // DETECTED_OBJECT_HPP
