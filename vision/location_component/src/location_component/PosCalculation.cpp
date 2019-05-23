#include "location_component/PosCalculation.hpp"

namespace location_component
{

  PosCalculation::PosCalculation()
  {
  }

  PosCalculation::~PosCalculation()
  {
  }

  cv::Point2f PosCalculation::calculateCupLocation(cv::Point aScreenPos)
  {
  }

  cv::Point2f PosCalculation::calculateAGVLocation(cv::Point aScreenPos)
  {
  }

  void PosCalculation::getWorldPos()
  {
    xpos*(100 / cAGVDepth) * cCupHeight;
    ypos*(100 / cAGVDepth) * cCupHeight;
  }

} // namespace location_component
