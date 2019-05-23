#ifndef POSTCALCULATION_HPP
#define POSTCALCULATION_HPP

const float cPosSensorX = 0.35;
const float cPosSensorY = -4.0;
const float cPosSensorZ = 1.8;

// size in cm
const float cCupHeight = 9.9;
const float cAGVDepth = 68.0;

const float cAGVWidth = 35.0;
const float cAGVHeight = 40.0;

namespace location_component
{

  class PosCalculation
  {
      public:
    PosCalculation();
    ~PosCalculation();
    cv::Point2f calculateCupLocation(cv::Point aScreenPos) cv::Point2f
        calculateAGVLocation(cv::Point aScreenPos) void getWorldPos();
  };
} // namespace location_component

#endif /* POSTCALCULATION_HPP */
