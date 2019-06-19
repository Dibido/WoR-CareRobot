#ifndef DETECTAGV_HPP
#define DETECTAGV_HPP

#include "location_component/AGV.hpp"
#include "location_component/AGVFrameCalibration.hpp"
#include "location_component/CupScanner.hpp"
#include "location_component/FrameCalibration.hpp"
#include "location_component/PosCalculation.hpp"
#include "location_component/RosServiceCup.hpp"

#include "location_component/DetectedAGV.hpp"
#include "location_component/DetectedFrame.hpp"

#include <boost/optional.hpp>
#include <iostream>
#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace location_component
{
  const unsigned int cCornersOfObject = 4;
  // The minimum size of a shape corner in order to be recognised by the polygon
  // detection algorithm.
  const unsigned int cEpsilon = 15;

  class DetectAGV
  {
      public:
    /**
     * @brief Creates a DetectAGV instance.
     *
     * @param aCalibration The cup detection calibration to be used.
     * @param aAGVFrameCalibration The AGV detection calibration to be used.
     */
    DetectAGV(CupDetectionCalibration& aCalibration,
              AGVFrameCalibration& aAGVFrameCalibration);
    /**
     * @brief Creates a DetectAGV instance.
     *
     * @param nh The node handle to be used for creating the cup location
     * publisher.
     * @param aCalibration The cup detection calibration to be used.
     * @param aAGVFrameCalibration The AGV detection calibration to be used.
     */
    DetectAGV(ros::NodeHandle& nh,
              CupDetectionCalibration& aCalibration,
              AGVFrameCalibration& aAGVFrameCalibration);
    ~DetectAGV();

    /**
     * @brief Calls the detectFrame function, and if an AGV and cups have been
     * detected, sends a message.
     */
    bool detectUpdate(const cv::Mat& aFrame, cv::Mat& aDisplayFrame);

    /**
     * @brief The detectFrame function will analyse the frame and decide what
     * the current position is of the AGV.
     *
     * @pre -
     * @post The captured view of the AGV/cups has been written to displayFrame.
     * If the AGV has crossed from one side of the screen to another, a new
     * capture and scan has been made.
     *
     * @param aFrame - This param will be the input of the function
     * @param aDisplayFrame - This reference matrix is used to display the debug
     * information of the detected objects
     */
    boost::optional<DetectedFrame> detectFrame(const cv::Mat& aFrame,
                                               cv::Mat& aDisplayFrame);

    /**
     * @brief This function will create a correct perspective image. If the
     * webcam is tilted it will correct the perspective.
     *
     * @param aFrame - This matrix is the input image of the AGV
     * @return boost::optional<DetectedAGV> - This param will only return if
     * there is an AGV detected. There can only be made a perspective image if
     * there is an AGV detected.
     */
    boost::optional<DetectedAGV> detect(const cv::Mat& aFrame) const;

    /**
     * @brief This function will transform the matrix with transformation
     * information.
     *
     * @param aTransmtx - The transformation matrix that determines how to
     * transform the matrix. See
     * https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
     * @param aSourceMat - The input matrix.
     * @param aDist - The output matrix after the transformation has been done.
     */
    void makePerspectiveCorrection(const cv::Mat& aTransmtx,
                                   const cv::Mat& aSourceMat,
                                   cv::Mat& aDist) const;

    /**
     * @brief Get the Contours Mat object
     *
     * @param aSourceMat - The matrix to get the contour of
     * @param aContours - A reference of a vector with all the found points of
     * the contour
     */
    void getContourMat(const cv::Mat& aSourceMat,
                       std::vector<cv::Point>& aContours) const;

    /**
     * @brief This function will return the midpoint of a contour aka a vector
     * of opencv points.
     *
     * @param aContours - The contours of a object
     * @return cv::Point - The midpoint of all the different points.
     */
    cv::Point getMidPoint(const std::vector<cv::Point>& aContours) const;

    /**
     * @brief Get agv speed.
     *
     * @return - The type of the return value is float. The speed is meter per
     * second.
     *
     */
    float getAGVSpeed() const;

    /**
     * @brief This function will pass through the speed of the AVG to the
     * PosCalculation class
     *
     * @param aSpeed - The current speed of the AGV
     */
    void setAGVSpeed(const location_component::AGV& aAGV);

    /**
     * @brief This function will set the variable mDetectObject
     *
     */
    void setDetectObject(bool aDetectObject);

      private:
    boost::optional<DetectedAGV> mPrevDetectedAGV;
    cv::Mat mCapturedFrame;
    std::unique_ptr<RosServiceCup> mRosServiceCup;
    location_component::PosCalculation mPosCalculator;
    CupDetectionCalibration mCalibration;
    FrameCalibration mFrameCalibration;
    bool mDetectObject = false;
  };
} // namespace location_component

#endif
