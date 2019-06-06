#include "location_component/FrameCalibration.hpp"
#include "opencv2/core.hpp"

#include "opencv2/core/utility.hpp"

#include "opencv2/imgproc.hpp"

#include "opencv2/imgcodecs.hpp"

#include "opencv2/highgui.hpp"

namespace location_component
{

  FrameCalibration::FrameCalibration(AGVFrameCalibration aAGVFrameCalibration)
      : mAGVFrameCalibration(aAGVFrameCalibration)
  {
  }

  void gammaCorrection(const cv::Mat& aSource,
                       cv::Mat& aDestination,
                       float aGamma)
  {
    cv::Mat lLabImage;
    cv::cvtColor(aSource, lLabImage, CV_BGR2Lab);
    std::vector<cv::Mat> lLabPlanes(3);
    cv::split(lLabImage, lLabPlanes);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat lClaheDst;
    clahe->apply(lLabPlanes[0], lClaheDst);
    lClaheDst.copyTo(lLabPlanes[0]);
    cv::merge(lLabPlanes, lLabImage);
    std::cout << "AAAAAAAAAA" << std::endl;
    cv::cvtColor(lLabImage, aDestination, CV_Lab2BGR);

    /* // build look up table */
    /* unsigned char lut[256]; */
    /* for (int i = 0; i < 256; i++) */
    /* { */
    /*   lut[i] = cv::saturate_cast<unsigned char>( */
    /*       pow(( float )(i / 255.0), aGamma) * 255.0f); */
    /* } */
    /* aDestination = aSource.clone(); */
    /* const int channels = aDestination.channels(); */
    /* switch (channels) */
    /* { */
    /* case 1: */
    /* { */
    /*   cv::MatIterator_<unsigned char> it, end; */
    /*   for (it = aDestination.begin<unsigned char>(), */
    /*       end = aDestination.end<unsigned char>(); */
    /*        it != end; it++) */
    /*     // *it = pow((float)(((*it))/255.0), aGamma) * 255.0; */
    /*     *it = lut[(*it)]; */
    /*   break; */
    /* } */
    /* case 3: */
    /* { */
    /*   cv::MatIterator_<cv::Vec3b> it, end; */
    /*   for (it = aDestination.begin<cv::Vec3b>(), */
    /*       end = aDestination.end<cv::Vec3b>(); */
    /*        it != end; it++) */
    /*   { */
    /*     (*it)[0] = lut[((*it)[0])]; */
    /*     (*it)[1] = lut[((*it)[1])]; */
    /*     (*it)[2] = lut[((*it)[2])]; */
    /*   } */
    /*   break; */
    /* } */
    /* } */
  }

  void FrameCalibration::removeEverythingButAGV(const cv::Mat& aSource,
                                                cv::Mat& aDestination) const
  {
    cv::Mat lCompensatedSource;
    cv::Mat lSourceHSV;
    cv::cvtColor(aSource, lSourceHSV, CV_BGR2HSV);
    
    /* gammaCorrection(aSource, lCompensatedSource, 1.0); */

    aSource.copyTo(lCompensatedSource);
    cv::Scalar lScalarLow(mAGVFrameCalibration.cHLow,
                          mAGVFrameCalibration.cSLow,
                          mAGVFrameCalibration.cVLow);

    cv::Scalar lScalarHigh(mAGVFrameCalibration.cHHigh,
                           mAGVFrameCalibration.cSHigh,
                           mAGVFrameCalibration.cVHigh);

    cv::inRange(lSourceHSV, lScalarLow, lScalarHigh, aDestination);

    if (mAGVFrameCalibration.mDebugStatus)
    {
      cv::Mat aLeftImg;
      cv::Mat aRightImg;
      cv::Mat aDestinationChannels[] = { aDestination, aDestination,
                                         aDestination };
      cv::Mat aDestinationRGB;
      cv::merge(aDestinationChannels, 3, aDestinationRGB);
      cv::pyrDown(lCompensatedSource, aLeftImg);
      cv::pyrDown(aDestinationRGB, aRightImg);
      cv::Mat aDebugImg;
      cv::hconcat(aLeftImg, aRightImg, aDebugImg);

      cv::imshow("debug", aDebugImg);
    }
  }

} // namespace location_component
