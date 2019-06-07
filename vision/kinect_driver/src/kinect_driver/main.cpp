#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  // Setup OpenCV
  const std::string mainWinName = "KinectImage";
  cv::namedWindow(mainWinName, cv::WINDOW_AUTOSIZE);
  // Set logger
  libfreenect2::setGlobalLogger(
      libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

  // Open the device
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device* dev = 0;
  libfreenect2::PacketPipeline* pipeline = 0;
  std::string serial = "";
  if (freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }
  if (serial == "")
  {
    serial = freenect2.getDefaultDeviceSerialNumber();
  }
  dev = freenect2.openDevice(serial);

  // Configure the device
  bool enable_rgb = true;
  bool enable_depth = false;
  int types = 0;
  if (enable_rgb)
    types |= libfreenect2::Frame::Color;
  if (enable_depth)
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;
  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

  // Start the device
  if (enable_rgb && enable_depth)
  {
    if (!dev->start())
      return -1;
  }
  else
  {
    if (!dev->startStreams(enable_rgb, enable_depth))
      return -1;
  }
  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  libfreenect2::Registration* registration = new libfreenect2::Registration(
      dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  // Get frames
  while (true)
  {
    if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 seconds
    {
      std::cout << "timeout!" << std::endl;
      return -1;
    }
    libfreenect2::Frame* rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];
    registration->apply(rgb, depth, &undistorted, &registered);
    // Send image over ROS topic
    cv::Mat lRgbMat(rgb->height, rgb->width, CV_8UC3, rgb->data);
    // cv::Mat lRgbImage = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
    cv::imshow(mainWinName, lRgbMat);
    // Release the frame
    listener.release(frames);
    cv::waitKey();
  }
  dev->stop();
  dev->close();
  return 0;
}