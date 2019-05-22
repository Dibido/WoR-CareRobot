#ifndef DETECTAGV_HPP
#define DETECTAGV_HPP

#include <boost/optional.hpp>
#include <iostream>
#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace location_component
{
  const unsigned int CornersOfObject = 4;
  const unsigned int CenterMargin = 50; // Pixels
  const float AngleMargin = 10;
  const float AngleRect = -90;

  struct DetectedAGV
  {
    std::vector<cv::Point> mCorners;
    cv::Point mMidpoint;
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
     * @param frame - This param will be the input of the function
     * @param displayFrame - This reference matrix is used to display the debug
     * information of the detected objects
     */
    void detectFrame(const cv::Mat& frame, cv::Mat& displayFrame);

    /**
     * @brief This function will create a correct perspective image. If the
     * webcam is tilted it will correct the perspective.
     *
     * @param frame - This matrix is the input image of the agv
     * @return boost::optional<DetectedAGV> - This param will only return if
     * there is an AGV detected. There can only be made a perspective image if
     * there is an AGV detected.
     */
    boost::optional<DetectedAGV> detect(const cv::Mat& frame) const;

    /**
     * @brief This function will transform the matrix with transformation
     * information.
     *
     * @param transmtx - The transformation matrix that determines how to
     * transform the matrix. See
     * https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
     * @param sourceMat - The input matrix.
     * @param dist - The output matrix after the transformation has been done.
     */
    void makePerspectiveCorrection(const cv::Mat& transmtx,
                                   const cv::Mat& sourceMat,
                                   cv::Mat& dist) const;

    /**
     * @brief Get the Contours Mat object
     *
     * @param sourceMat - The matrix to get the contours of
     * @param contours - A reference of a vector with all the found contours
     */
    void getContoursMat(const cv::Mat& sourceMat,
                        std::vector<std::vector<cv::Point>>& contours) const;

    /**
     * @brief This function will return the midpoint of a contour aka a vector
     * of opencv points.
     *
     * @param contours - The contours of a object
     * @return cv::Point - The midpoint of all the different points.
     */
    cv::Point getMidPoint(const std::vector<cv::Point>& contours) const;

      private:
    boost::optional<DetectedAGV> prevDetectedAGV;
    cv::Mat capturedFrame;
  };
} // namespace location_component

#endif
