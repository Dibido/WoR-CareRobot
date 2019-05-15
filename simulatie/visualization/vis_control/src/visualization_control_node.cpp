
#include <ros/ros.h>

#include <vis_control/Visualizer.hpp>

// Include visualizers
#include <vis_control/DetectedObjectVisualizer.hpp>
#include <vis_control/KinectImageVisualizer.hpp>
#include <vis_control/KinectPointsVisualizer.hpp>
#include <vis_control/LidarVisualizer.hpp>
#include <vis_control/PathVisualizer.hpp>

// Include messages
#include <nav_msgs/Path.h>
#include <path_planner/Coordinate.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision/DetectedObjects.h>
#include <visualization_msgs/MarkerArray.h>

using namespace visualization;

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle nodeHandle("~");
  ros::Rate rate(100);

  // Get the subscribe topic names
  std::string visionObjIn = "", kinectDepthIn = "", kinectImgIn = "",
              lidarScanIn = "", pathIn = "";
  nodeHandle.getParam("kinect_depth_in", kinectDepthIn);
  nodeHandle.getParam("kinect_image_in", kinectImgIn);
  nodeHandle.getParam("vision_object_in", visionObjIn);
  nodeHandle.getParam("lidar_scan_in", lidarScanIn);
  nodeHandle.getParam("path_in", pathIn);

  // Get the frame names
  std::string kinectFrame = "", lidarFrame = "", visionFrame = "",
              pathFrame = "";
  nodeHandle.getParam("kinect_frame", kinectFrame);
  nodeHandle.getParam("vision_object_frame", visionFrame);
  nodeHandle.getParam("lidar_frame", lidarFrame);
  nodeHandle.getParam("path_frame", pathFrame);

  // Initialize the visualizers
  LidarVisualizer lidarVisualizer(lidarFrame);
  KinectPointsVisualizer kinectPointsVisualizer(kinectFrame);
  KinectImageVisualizer kinectImageVisualizer(kinectFrame);
  DetectedObjectVisualizer visionObjectVisualizer(visionFrame);
  PathVisualizer pathVisualizer(pathFrame);

  // Advertise topics for RViz
  auto lidarDataPublisher =
      nodeHandle.advertise<sensor_msgs::LaserScan>("lidar", 1);
  auto kinectPointcloudPublisher =
      nodeHandle.advertise<sensor_msgs::PointCloud2>("kinect/points", 1);
  auto kinectImagePublisher =
      nodeHandle.advertise<sensor_msgs::Image>("kinect/image_raw", 1);
  auto kinectObjectPublisher =
      nodeHandle.advertise<visualization_msgs::MarkerArray>("kinect/object", 1);
  auto pathPublisher = nodeHandle.advertise<nav_msgs::Path>("path", 1);

  // Initialize publish lambdas
  LidarVisualizer::publishFn_t publishLidarDataFn =
      [&lidarDataPublisher](const sensor_msgs::LaserScan &message) {
        lidarDataPublisher.publish(message);
      };
  KinectPointsVisualizer::publishFn_t publishKinectPointcloudFn =
      [&kinectPointcloudPublisher](const sensor_msgs::PointCloud2 &message) {
        kinectPointcloudPublisher.publish(message);
      };
  KinectImageVisualizer::publishFn_t publishKinectImageFn =
      [&kinectImagePublisher](const sensor_msgs::Image &message) {
        kinectImagePublisher.publish(message);
      };
  DetectedObjectVisualizer::publishFn_t publishKinectObjectsFn =
      [&kinectObjectPublisher](const visualization_msgs::MarkerArray &message) {
        kinectObjectPublisher.publish(message);
      };
  PathVisualizer::publishFn_t publishPathFn =
      [&pathPublisher](const nav_msgs::Path &message) {
        pathPublisher.publish(message);
      };

  // Connect publish lambdas to their classes
  lidarVisualizer.setPublishFn(publishLidarDataFn);
  kinectPointsVisualizer.setPublishFn(publishKinectPointcloudFn);
  kinectImageVisualizer.setPublishFn(publishKinectImageFn);
  visionObjectVisualizer.setPublishFn(publishKinectObjectsFn);
  pathVisualizer.setPublishFn(publishPathFn);

  // Connect callback methods to topics
  auto lidarSub = nodeHandle.subscribe(
      lidarScanIn, 10, &LidarVisualizer::callback, &lidarVisualizer);
  auto kinectPointcloudSub =
      nodeHandle.subscribe(kinectDepthIn, 10, &KinectPointsVisualizer::callback,
                           &kinectPointsVisualizer);
  auto kinectImageSub =
      nodeHandle.subscribe(kinectImgIn, 10, &KinectImageVisualizer::callback,
                           &kinectImageVisualizer);
  auto kinectObjectsSub =
      nodeHandle.subscribe(visionObjIn, 10, &DetectedObjectVisualizer::callback,
                           &visionObjectVisualizer);
  auto pathSub = nodeHandle.subscribe(pathIn, 10, &PathVisualizer::callback,
                                      &pathVisualizer);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
