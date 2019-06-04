#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

namespace location_component
{
  struct CupDetectionCalibration
  {
    // Camera position
    float mCameraPosX_m = 0.35f;
    float mCameraPosY_m = -4.0f;
    float mCameraPosZ_m = 0.8f;

    // Camera FOV
    float mCameraFOV_rads = 1.047f;

    // The position where the arm will pick up the cup
    float mArmY_m = 0.0f;

    // Compensate for flipped camera
    float mCameraFlipX = -1.0f;
    float mCameraFlipY = -1.0f;

    // Cup and AGV size
    float mCupHeight_m = 0.099f;
    float mCupDiameter_m = 0.071f;
    float mAGVDepth_m = 0.680f;
    float mAGVWidth_m = 0.350f;
    float mAGVHeight_m = 0.400f;
  };

  struct AGVFrameCalibration
  {
    //Color spectrum AGV low
    const float cHLow = 0.0;
    const float cSLow = 0.0;
    const float cVLow = 0.0;

    //Color spectrum AGV high
    const float cHHigh = 255.0;
    const float cSHigh = 255.0;
    const float cVHigh = 30.0;

  };
} // namespace location_component

#endif /* CALIBRATION_HPP */
