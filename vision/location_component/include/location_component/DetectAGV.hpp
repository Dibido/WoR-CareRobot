#ifndef DETECTAGV_HPP
#define DETECTAGV_HPP

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
    cv::Mat agvFrame;
  };

  class DetectAGV
  {
      public:
    DetectAGV();
    ~DetectAGV();

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
    void detectFrame(const cv::Mat& aFrame, cv::Mat& aDisplayFrame);

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
  };
} // namespace location_component

#endif
