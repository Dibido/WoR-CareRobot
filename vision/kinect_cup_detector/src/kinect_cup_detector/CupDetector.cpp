#include "kinect_cup_detector/CupDetector.hpp"

CupDetector::CupDetector(bool aDebugMode)
    : mDebugMode(aDebugMode), mCalibrated(false), mSendGoal(false)
{
  // Setup opencv
  if (mDebugMode)
  {
    cv::namedWindow("result");
  }
  image_transport::ImageTransport mImageTransport(mNodeHandle);
  image_transport::Publisher mPublisher;
  // Read from ROS topic
  // Subscribe to the goal topic for receaving the user prefrence
  mGoalSubscriber = mNodeHandle.subscribe(kinect_cupdetector::cGoalTopic, 1,
                                          &CupDetector::goalCallback, this);
  mSubscriber = mImageTransport.subscribe(kinect_cupdetector::cKinectImageTopic,
                                          1, &CupDetector::imageCallBack, this);
  mCupPublisher = mNodeHandle.advertise<kinematica_msgs::Cup>(
      environment_controller::cCupTopicName, 1000);
}

void CupDetector::calibrateKinectPosition()
{
  // Calibrate the center of the kinect image  to be 70 cm away from the
  // robotarm.
  // Draw a cirlce in the center of the image
  cv::Scalar lRedColor = cv::Scalar(0, 0, 255);
  cv::circle(mDisplayMatrix,
             cv::Point(mDisplayMatrix.cols / 2, mDisplayMatrix.rows / 2), 4,
             lRedColor, 2);
  // Draw a line across the center of the image
  cv::line(mDisplayMatrix, cv::Point((mDisplayMatrix.cols / 2), 0),
           cv::Point((mDisplayMatrix.cols / 2), mDisplayMatrix.rows), lRedColor,
           4);
  if (mDebugMode)
  {
    cv::imshow("result", mDisplayMatrix);
  }
}

void CupDetector::goalCallback(const kinematica_msgs::GoalConstPtr& aMsg)
{
  // If the static goal is requested we send it
  try
  {
    if (aMsg->staticCup)
    {
      mSendGoal = true;
    }
  }
  catch (const std::exception& lE)
  {
    ROS_ERROR("%s", lE.what());
  }
}

void CupDetector::imageCallBack(const sensor_msgs::ImageConstPtr& aMsg)
{
  // Try to convert the image from the ros message
  try
  {
    cv_bridge::toCvShare(aMsg, "bgr8")->image.copyTo(mDisplayMatrix);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", aMsg->encoding.c_str());
  }

  if (!mCalibrated)
  {
    // Calibrate the position of the kinect
    calibrateKinectPosition();
    if (mDebugMode)
    {
      int c = cv::waitKey(10);
      if (c == 27) // Escape
      {
        mCalibrated = true;
      }
      else
      {
        return;
      }
    }
  }

  // Filter color
  cv::cvtColor(mDisplayMatrix, mDisplayHSV, CV_RGB2HSV);
  cv::inRange(mDisplayHSV, kinect_cupdetector::cMinHSVRectangeValues,
              kinect_cupdetector::cMaxHSVRectangeValues, mColorMask);
  // Find contours
  cv::findContours(mColorMask, mImageContours, mImageHierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  // Find the correct contour
  for (unsigned int i = 0; i < mImageContours.size(); i++)
  {
    double lEpsilon = kinect_cupdetector::cEpsilonMultiply *
                      cv::arcLength(mImageContours.at(i), true);
    approxPolyDP(mImageContours.at(i), mApproxImage, lEpsilon, true);
    if (mApproxImage.size().height ==
            kinect_cupdetector::cRectangleCornercount &&
        (contourArea(mImageContours.at(i)) >
         kinect_cupdetector::cMinRectangleContourSize))
    {
      // Get the number of pixels per cm (longest side of rectangle is 29cm)
      mRotatedRect = cv::minAreaRect(mImageContours.at(i));
      // Get the biggest side
      double lMaxDistance = 0;

      mRotatedRect.points(mRectangleVertices);
      for (int i = 0; i < 4; i++)
      {
        double lDistance = ( double )cv::norm(mRectangleVertices[i] -
                                              mRectangleVertices[(i + 1) % 4]);
        if (lDistance > lMaxDistance)
        {
          lMaxDistance = ( double )lDistance;
        }
      }
      double lPixelsPerCm =
          lMaxDistance / kinect_cupdetector::cRectangleLongestSideLength;
      // Get the center of the rectangle in the frame
      // Calculate center
      cv::Moments lRectangleMoments = cv::moments(mImageContours.at(i));
      mCenterPaperX = ( int )(lRectangleMoments.m10 / lRectangleMoments.m00);
      mCenterPaperY = ( int )(lRectangleMoments.m01 / lRectangleMoments.m00);

      // Find Cup on rectangle
      mRegionOfInterest =
          mDisplayMatrix(cv::boundingRect(mImageContours.at(i)));
      cv::cvtColor(mRegionOfInterest, mRegionOfInterestHSV, CV_RGB2HSV);
      cv::inRange(
          mRegionOfInterestHSV, kinect_cupdetector::cMinHSVRectangeValues,
          kinect_cupdetector::cMaxHSVRectangeValues, mRegionOfInterestMask);
      cv::findContours(mRegionOfInterestMask, mRegionOfInterestContours,
                       mImageHierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_TC89_L1);
      for (unsigned int j = 0; j < mRegionOfInterestContours.size(); j++)
      {
        if (contourArea(mRegionOfInterestContours.at(j)) >
            kinect_cupdetector::cMinCupContourSize)
        {
          approxPolyDP(mRegionOfInterestContours.at(j), mApproxImageCup,
                       lEpsilon, true);
          // Get the center of the contour
          cv::Moments lCupMoments =
              cv::moments(mRegionOfInterestContours.at(j));
          mCenterCupX = ( int )(lCupMoments.m10 / lCupMoments.m00);
          mCenterCupY = ( int )(lCupMoments.m01 / lCupMoments.m00);
          // Calculate difference from the center of the paper
          mCenterCupX = (mRegionOfInterest.cols / 2) - mCenterCupX;
          mCenterCupY = (mRegionOfInterest.rows / 2) - mCenterCupY;
        }
      }
      // Determine the cup position relative to the center of the paper
      int lCupPaperPositionX = mCenterPaperX - mCenterCupX;
      int lCupPaperPositionY = mCenterPaperY - mCenterCupY;
      // Determine position relative to the middle of the screen
      int distFromCenterX = (mDisplayMatrix.cols / 2) - lCupPaperPositionX;
      int distFromCenterY = (mDisplayMatrix.rows / 2) - lCupPaperPositionY;
      // Convert the distances from pixels to centimeters
      double distFromCenterXCM = distFromCenterX / lPixelsPerCm;
      double distFromCenterYCM = distFromCenterY / lPixelsPerCm;
      // Subtract the gripper size from the Y position
      distFromCenterXCM -= kinect_cupdetector::cCupGrippperSize / 2;
      // Add the width of the gripper to the end position
      distFromCenterYCM += kinect_cupdetector::cCupGrippperWidth;
      // Subtract the calibrated distance from the robotarm
      distFromCenterYCM -= kinect_cupdetector::cYDistanceFromRobotarm;
      // Invert the Y since the robotarm is on the other side of the kinect
      distFromCenterYCM = -distFromCenterYCM;
      if (mSendGoal)
      {
        kinematica_msgs::Cup lFoundCup;
        lFoundCup.aDepth = 0.07;
        lFoundCup.aDirection = 0;
        lFoundCup.aHeight = 0.082;
        lFoundCup.aMeasurementTime = ros::Time::now();
        lFoundCup.aSensorId = 1;
        lFoundCup.aSpeed = 0;
        lFoundCup.aWidth = 0.061;
        // Since the kinect is positioned oposite of the robotarm so X=Y and
        // Y=X.
        lFoundCup.mX_m = distFromCenterYCM / 100;
        lFoundCup.mY_m = distFromCenterXCM / 100;
        lFoundCup.mZ_m = 0.02;
        lFoundCup.timeOfArrival = ros::Time::now();
        mCupPublisher.publish(lFoundCup);
        ros::spinOnce();
        // we sent the goal, now we wait
        std::cout << "!!! --- SENT THE CUP POSITION --- !!!" << std::endl;
        mSendGoal = false;
      }
      std::cout << "FoundX : " << distFromCenterYCM / 100
                << "FoundY : " << distFromCenterXCM / 100 << std::endl;
    }
  }
  // Show in a window
  cv::cvtColor(mDisplayHSV, mDisplayMatrix, cv::COLOR_HSV2RGB);
  // cv::rectangle(mDisplayMatrix, mRotatedRect.tl(), mRotatedRect.br(),
  // cv::Scalar(0, 0, 255), 2, 8, 0);
  for (int i = 0; i < 4; i++)
  {
    cv::line(mDisplayMatrix, mRectangleVertices[i],
             mRectangleVertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
  }
  cv::circle(
      mDisplayMatrix,
      cv::Point(mCenterPaperX - mCenterCupX, mCenterPaperY - mCenterCupY), 4,
      cv::Scalar(0, 0, 255), 3);
  if (mDebugMode)
  {
    cv::imshow("result", mDisplayMatrix);
  }
  int c = cv::waitKey(10);
  if (c == 27) // Escape
  {
    std::exit(0);
  }
}