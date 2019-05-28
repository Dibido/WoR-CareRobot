#ifndef DETECTAGV_HPP
#define DETECTAGV_HPP

#include "location_component/Calibration.hpp"
#include "location_component/CupScanner.hpp"
#include "location_component/RosServiceCup.hpp"
#include <boost/optional.hpp>
#include <iostream>
#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace location_component
{
  const unsigned int cCornersOfObject = 4;

  struct DetectedAGV
  {
    std::vector<cv::Point> mCorners;
    cv::Point mMidpoint;
    cv::Rect mBoundRect;
    cv::Mat mAGVFrame;
  };

  struct DetectedFrame
  {
    DetectedAGV mDetectedAGV;
    std::vector<DetectedCup> mDetectedCups;
    cv::Size mAGVFrameSize;
    cv::Size mCupFrameSize;
  };

  class DetectAGV
  {
      public:
    DetectAGV(Calibration aCalibration = Calibration());
    DetectAGV(ros::NodeHandle& nh, Calibration aCalibration = Calibration());
    ~DetectAGV();

    /**
     * @brief Calls the detectFrame function, and if an AGV and cups have been
     * detected, sends a message.
     */
    void detectUpdate(const cv::Mat& aFrame, cv::Mat& aDisplayFrame);

    /**
     * @brief The detectFrame function will analyse the frame and decide what
     * the current position is of the agv.
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
     * @param aFrame - This matrix is the input image of the agv
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
     * @param aSourceMat - The matrix to get the contours of
     * @param aContours - A reference of a vector with all the found contours
     */
    void getContoursMat(const cv::Mat& aSourceMat,
                        std::vector<std::vector<cv::Point>>& aContours) const;

    /**
     * @brief This function will return the midpoint of a contour aka a vector
     * of opencv points.
     *
     * @param aContours - The contours of a object
     * @return cv::Point - The midpoint of all the different points.
     */
    cv::Point getMidPoint(const std::vector<cv::Point>& aContours) const;

      private:
    boost::optional<DetectedAGV> mPrevDetectedAGV;
    cv::Mat mCapturedFrame;
    std::unique_ptr<RosServiceCup> mRosServiceCup;
    Calibration mCalibration;
  };
} // namespace location_component

#endif
