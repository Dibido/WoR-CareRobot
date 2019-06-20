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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  // Setup ROS
  ros::init(argc, argv, "kinect_driver");
  ros::NodeHandle lNodeHandle;
  image_transport::ImageTransport mImageTransport(lNodeHandle);
  image_transport::Publisher mPublisher;

  mPublisher = mImageTransport.advertise("/sensor/kinect/img_raw", 1);

  // Set logger
  libfreenect2::setGlobalLogger(
      libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

  // Open the device
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device* dev = 0;
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
  ros::Rate lRateLimit(10); // 10 hz
  while (ros::ok())
  {
    // Ratelimit to 10Hz

    if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 seconds
    {
      std::cout << "timeout!" << std::endl;
      return -1;
    }
    libfreenect2::Frame* rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];
    registration->apply(rgb, depth, &undistorted, &registered);
    // Send image over ROS topic
    cv::Mat bgr[4];
    cv::Mat rgbmat;
    cv::Mat(static_cast<int>(rgb->height), static_cast<int>(rgb->width),
            CV_8UC4, rgb->data)
        .copyTo(rgbmat);
    split(rgbmat, bgr);
    // Only using one channel to detect the marker
    std::vector<cv::Mat> channels = { bgr[0], bgr[1], bgr[2] };
    cv::Mat img_original;
    merge(channels, img_original);
    sensor_msgs::ImagePtr lMsg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_original)
            .toImageMsg();
    mPublisher.publish(lMsg);
    // Release the frame
    listener.release(frames);

    lRateLimit.sleep();
  }
  dev->stop();
  dev->close();
  return 0;
}