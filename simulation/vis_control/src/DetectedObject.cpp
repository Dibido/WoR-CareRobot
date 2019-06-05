
#include <vis_control/DetectedObject.hpp>

namespace visualization {

DetectedObject::DetectedObject(size_t index, double x, double y, double z,
                               double width, double height, double depth)
    : index(index), x(x), y(y), z(z), width(width), height(height),
      depth(depth) {}

size_t DetectedObject::getIndex() const { return index; }

double DetectedObject::getX() const { return x; }

double DetectedObject::getY() const { return y; }

double DetectedObject::getZ() const { return z; }

double DetectedObject::getWidth() const { return width; }

double DetectedObject::getHeight() const { return height; }

double DetectedObject::getDepth() const { return depth; }

} // namespace visualization