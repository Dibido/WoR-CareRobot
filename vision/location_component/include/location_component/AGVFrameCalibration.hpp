#ifndef AGV_FRAME_CALIBRATION_HPP
#define AGV_FRAME_CALIBRATION_HPP

namespace location_component
{

  struct AGVFrameCalibration
  {
    /**
     * @brief Construct a new AGVFrameCalibration object
     *
     * @param aDebugStatus - This value is used to show the debug status of the
     * AGV frame calibration
     */
    AGVFrameCalibration(bool aDebugStatus);
    // Color spectrum AGV low
    const double cHLow = 70.0;
    const double cSLow = 130.0;
    const double cVLow = 70.0;

    // Color spectrum AGV high
    const double cHHigh = 170.0;
    const double cSHigh = 255.0;
    const double cVHigh = 255.0;

    bool mDebugStatus;
  };
} // namespace location_component

#endif
