#ifndef WEBCAMPUBLISHER_HPP
#define WEBCAMPUBLISHER_HPP

#include "webcam_driver/WebcamDriver.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

namespace webcam_driver
{
  class WebcamPublisher
  {
      public:
    /**
     * @brief Creates a WebcamPublisher class.
     *        This class reads frames from a WebcamDriver
     *        and publishes the image data on a ROS topic.
     * @param aWebcamId The webcam ID.
     * @param aDebug Whether to display a debug window with the webcam footage.
     * @param aTopicName The name of the ROS topic to publish on.
     * @param aNH The ROS node handle to be used.
     */
    WebcamPublisher(unsigned int aWebcamId,
                    bool aDebug,
                    const std::string& aTopicName,
                    ros::NodeHandle& aNH);
    ~WebcamPublisher();
    /**
     * @brief Reads a frame from the webcam driver and publishes it on the
     *        topic.
     */
    void update();

      private:
    webcam_driver::WebcamDriver mWebcamDriver;
    image_transport::ImageTransport mImageTransport;
    image_transport::Publisher mPublisher;
    bool mDebug;
  };
} // namespace webcam_driver

#endif /* WEBCAMPUBLISHER_HPP */
