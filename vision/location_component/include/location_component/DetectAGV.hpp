#ifndef DETECTAGV_HPP
#define DETECTAGV_HPP

#include <boost/optional.hpp>
#include <iostream>
#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

const unsigned int CornersOfObject = 4;
const unsigned int CenterMargin = 50; // Pixels
const float AngleMargin = 10;
const float AngleRect = -90;

namespace location_component
{
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
     * @return boost::optional<DetectedAGV> -This param will only return of
     * there is a AGV detected. There can only be made a perspective image of
     * the is a AGV detected.
     */
    boost::optional<DetectedAGV> detect(const cv::Mat& frame);

    /**
     * @brief This function will transform the matrix with transformation
     * information.
     *
     * @param transmtx - The transformation matrix that determines how to
     * transform the matrix.
     * @param sourceMat - The input matrix.
     * @param dist - The output matrix after the transformation has been done.
     */
    void makePerspectiveCorrection(const cv::Mat& transmtx,
                                   const cv::Mat& sourceMat,
                                   cv::Mat& dist);

    /**
     * @brief Get the Contours Mat object
     *
     * @param sourceMat - The matrix to get the contours of
     * @param contours - A reference of a vector with all the found contours
     */
    void getContoursMat(const cv::Mat& sourceMat,
                        std::vector<std::vector<cv::Point>>& contours);

    /**
     * @brief This function will return the midpoint of a contour aka a vector
     * of opencv points.
     *
     * @param contours - The contours of a object
     * @return cv::Point - The midpoint of all the different points.
     */
    cv::Point getMidPoint(std::vector<cv::Point>& contours);

      private:
    boost::optional<DetectedAGV> prevDetectedAGV;
    cv::Mat capturedFrame;
  };
} // namespace location_component

#endif
