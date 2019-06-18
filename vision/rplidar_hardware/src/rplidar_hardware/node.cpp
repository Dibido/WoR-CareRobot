/*
 *
 * This file houses the hardware implementation for the Lidar sensors. This file
 * has been refactored from the RPLidar SDK, found here:
 * https://github.com/Slamtec/rplidar_ros/blob/master/src/node.cpp
 *
 * The code doesn't comply with the coding guidelines, because the original code
 * doesn't and complex problems with this implementation limited the time we had
 * to build our own implementation from scratch.
 *
 */

#include "ros/ros.h"
#include "rplidar.h"
#include "sensor_interfaces/LidarData.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

/* This define houses _countof, which counts the size of an Array. This first
 * counts the size of the array in bytes, then it devides it by the size of one
 * element.
 */
#ifndef _countof
#define _countof(_Array) ( int )(sizeof(_Array) / sizeof(_Array[0]))
#endif

// Degree to radial define/function
#define DEG2RAD(x) (( x )*M_PI / 180.)

using namespace rp::standalone::rplidar;

RPlidarDriver* gDrv = NULL;

/**
 * @brief getAngle returns the calculated angle based on the raw values.
 *
 * @param node
 *
 * @return angle in a float
 */
static float getAngle(const rplidar_response_measurement_node_hq_t& node)
{
  return node.angle_z_q14 * 90.f / 16384.f;
}

/**
 * @brief publishScan publishes scanned data to the defined topic, in this case
 * "sensors/lidar"
 *
 * This function has been partly rewritten to output correct data for the
 * system, as the standard data was not accurate enough.
 */
void publish_scan(ros::Publisher* pub,
                  rplidar_response_measurement_node_hq_t* nodes,
                  size_t node_count,
                  ros::Time start,
                  double scan_time,
                  bool inverted,
                  float angle_min,
                  float angle_max,
                  float max_distance,
                  std::string frame_id)
{
  static int lScanCount = 0;
  sensor_msgs::LaserScan scan_msg;
  sensor_interfaces::LidarData lidar_msg;
  lidar_msg.header.stamp = start;
  lidar_msg.header.frame_id = frame_id;

  scan_msg.header.stamp = start;
  scan_msg.header.frame_id = frame_id;
  lScanCount++;

  bool reversed = (angle_max > angle_min);
  if (reversed)
  {
    scan_msg.angle_min = M_PI - angle_max;
    scan_msg.angle_max = M_PI - angle_min;
  }
  else
  {
    scan_msg.angle_min = M_PI - angle_min;
    scan_msg.angle_max = M_PI - angle_max;
  }
  scan_msg.angle_increment =
      (scan_msg.angle_max - scan_msg.angle_min) / ( double )(node_count - 1);

  for (size_t i = 0; i < node_count; ++i)
  {
    std::cout << "Angle: " << std::to_string(DEG2RAD(getAngle(nodes[i])))
              << std::endl;
  }

  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / ( double )(node_count - 1);
  scan_msg.range_min = 0.15;
  scan_msg.range_max = max_distance; // 8.0;

  scan_msg.intensities.resize(node_count);
  scan_msg.ranges.resize(node_count);
  lidar_msg.distances.resize(node_count);
  lidar_msg.measurement_angles.resize(node_count);
  bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
  if (!reverse_data)
  {
    for (size_t i = 0; i < node_count; i++)
    {
      float read_value = ( float )nodes[i].dist_mm_q2 / 4.0f / 1000;
      if (read_value == 0.0)
      {
        scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        lidar_msg.distances.push_back(0.0);
      }
      else
      {
        lidar_msg.measurement_angles.push_back(DEG2RAD(getAngle(nodes[i])));
        scan_msg.ranges[i] = read_value;
        lidar_msg.distances.push_back(read_value);
        scan_msg.intensities[i] = ( float )(nodes[i].quality >> 2);
      }
    }
  }
  else
  {
    for (size_t i = 0; i < node_count; i++)
    {
      float read_value = ( float )nodes[i].dist_mm_q2 / 4.0f / 1000;
      if (read_value == 0.0)
      {
        scan_msg.ranges[node_count - 1 - i] =
            std::numeric_limits<float>::infinity();
        lidar_msg.distances[node_count - 1 - i] = 0.0;
      }
      else
      {
        lidar_msg.measurement_angles[node_count - 1 - i] =
            DEG2RAD(getAngle(nodes[i]));
        scan_msg.ranges[node_count - 1 - i] = read_value;
        scan_msg.intensities[node_count - 1 - i] =
            ( float )(nodes[i].quality >> 2);
        lidar_msg.distances[node_count - 1 - i] = read_value;
      }
    }
  }

  pub->publish(lidar_msg);
}

/**
 * @brief checks Lidar device info
 *
 * @param gDrv
 *
 * @return false when there is a problem, otherwise return true.
 */
bool getRPLIDARDeviceInfo(RPlidarDriver* gDrv)
{
  u_result op_result;
  rplidar_response_device_info_t devinfo;

  op_result = gDrv->getDeviceInfo(devinfo);
  if (IS_FAIL(op_result))
  {
    if (op_result == RESULT_OPERATION_TIMEOUT)
    {
      ROS_ERROR("Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
    }
    else
    {
      ROS_ERROR("Error, unexpected error, code: %x", op_result);
    }
    return false;
  }

  // print out the device serial number, firmware and hardware version
  // number..
  printf("RPLIDAR S/N: ");
  for (int pos = 0; pos < 16; ++pos)
  {
    printf("%02X", devinfo.serialnum[pos]);
  }
  printf("\n");
  ROS_INFO("Firmware Ver: %d.%02d", devinfo.firmware_version >> 8,
           devinfo.firmware_version & 0xFF);
  ROS_INFO("Hardware Rev: %d", ( int )devinfo.hardware_version);
  return true;
}

/**
 * @brief checks health of Lidar hardware.
 *
 * @param gDrv
 *
 * @return false if there is a problem, otherwise return true.
 */
bool checkRPLIDARHealth(RPlidarDriver* gDrv)
{
  u_result op_result;
  rplidar_response_device_health_t healthinfo;

  op_result = gDrv->getHealth(healthinfo);
  if (IS_OK(op_result))
  {
    ROS_INFO("RPLidar health status : %d", healthinfo.status);
    if (healthinfo.status == RPLIDAR_STATUS_ERROR)
    {
      ROS_ERROR(
          "Error, rplidar internal error detected. Please reboot the device "
          "to retry.");
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    ROS_ERROR("Error, cannot retrieve rplidar health code: %x", op_result);
    return false;
  }
}

/**
 * @brief stops motor and returns status
 *
 * @param req
 * @param res
 *
 * @return
 */
bool stop_motor(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (!gDrv)
    return false;

  ROS_DEBUG("Stop motor");
  gDrv->stop();
  gDrv->stopMotor();
  return true;
}

bool start_motor(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (!gDrv)
    return false;
  ROS_DEBUG("Start motor");
  gDrv->startMotor();
  gDrv->startScan(0, 1);
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rplidar_node");

  std::string lPortPrefix = "/dev/";
  std::string lSerialPort = "ttyUSB0";

  // Serial port is given as argument
  if (argc == 2)
  {
    ROS_INFO("Setting serial port to: %s%s", lPortPrefix.c_str(), argv[1]);
    lSerialPort = argv[1];
  }

  std::string lFullSerialPort = lPortPrefix.append(lSerialPort);

  // Global variables
  std::string channel_type;
  std::string tcp_ip;
  std::string serial_port;
  int tcp_port = 20108;
  int serial_baudrate = 115200;
  std::string frame_id;
  bool inverted = false;
  bool angle_compensate = true;
  float max_distance = 8.0;
  int angle_compensate_multiple =
      1; // it stand of angle compensate at per 1 degree
  std::string scan_mode;
  ros::NodeHandle nh;
  ros::Publisher scan_pub =
      nh.advertise<sensor_interfaces::LidarData>("sensor/lidar", 1000);
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("channel_type", channel_type, "serial");
  nh_private.param<std::string>("tcp_ip", tcp_ip, "192.168.0.7");
  nh_private.param<int>("tcp_port", tcp_port, 20108);
  nh_private.param<std::string>("serial_port", serial_port, lFullSerialPort);
  nh_private.param<int>(
      "serial_baudrate", serial_baudrate,
      115200 /*256000*/); // ros run for A1 A2, change to 256000 if A3
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  nh_private.param<bool>("inverted", inverted, false);
  nh_private.param<bool>("angle_compensate", angle_compensate, false);
  nh_private.param<std::string>("scan_mode", scan_mode, std::string());
  u_result op_result;

  // Create the driver instance
  if (channel_type == "tcp")
  {
    gDrv =
        RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP);
  }
  else
  {
    gDrv = RPlidarDriver::CreateDriver(
        rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
  }

  if (!gDrv)
  {
    ROS_ERROR("Create Driver fail, exit");
    return -2;
  }

  if (channel_type == "tcp")
  {
    // make connection...
    if (IS_FAIL(gDrv->connect(tcp_ip.c_str(), ( _u32 )tcp_port)))
    {
      ROS_ERROR("Error, cannot bind to the specified serial port %s.",
                serial_port.c_str());
      RPlidarDriver::DisposeDriver(gDrv);
      return -1;
    }
  }
  else
  {
    // make connection...
    if (IS_FAIL(gDrv->connect(serial_port.c_str(), ( _u32 )serial_baudrate)))
    {
      ROS_ERROR("Error, cannot bind to the specified serial port %s.",
                serial_port.c_str());
      RPlidarDriver::DisposeDriver(gDrv);
      return -1;
    }
  }

  // get rplidar device info
  if (!getRPLIDARDeviceInfo(gDrv))
  {
    return -1;
  }

  // check health...
  if (!checkRPLIDARHealth(gDrv))
  {
    RPlidarDriver::DisposeDriver(gDrv);
    return -1;
  }

  //  ros::ServiceServer stop_motor_service =
  //      nh.advertiseService("stop_motor", stop_motor);
  //  ros::ServiceServer start_motor_service =
  //      nh.advertiseService("start_motor", start_motor);

  gDrv->startMotor();

  /*
   * This section checks which scan types the sensor supports, and starts a
   * scan. In our case, the express scan is used.
   */
  RplidarScanMode current_scan_mode;
  if (scan_mode.empty())
  {
    op_result = gDrv->startScan(false /* not force scan */,
                                true /* use typical scan mode */, 0,
                                &current_scan_mode);
  }
  else
  {
    std::vector<RplidarScanMode> allSupportedScanModes;
    op_result = gDrv->getAllSupportedScanModes(allSupportedScanModes);

    if (IS_OK(op_result))
    {
      _u16 selectedScanMode = _u16(-1);
      for (std::vector<RplidarScanMode>::iterator iter =
               allSupportedScanModes.begin();
           iter != allSupportedScanModes.end(); iter++)
      {
        if (iter->scan_mode == scan_mode)
        {
          selectedScanMode = iter->id;
          break;
        }
      }

      if (selectedScanMode == _u16(-1))
      {
        ROS_ERROR("scan mode `%s' is not supported by lidar, supported modes:",
                  scan_mode.c_str());
        for (std::vector<RplidarScanMode>::iterator iter =
                 allSupportedScanModes.begin();
             iter != allSupportedScanModes.end(); iter++)
        {
          ROS_ERROR("\t%s: max_distance: %.1f m, Point number: %.1fK",
                    iter->scan_mode, iter->max_distance,
                    (1000 / iter->us_per_sample));
        }
        op_result = RESULT_OPERATION_FAIL;
      }
      else
      {
        op_result =
            gDrv->startScanExpress(false /* not force scan */, selectedScanMode,
                                   0, &current_scan_mode);
      }
    }
  }

  /*
   * This part parses scanned data
   */

  if (IS_OK(op_result))
  {
    // default frequent is 10 hz (by motor pwm value),
    // current_scan_mode.us_per_sample is the number of scan point per us
    angle_compensate_multiple =
        ( int )(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
    if (angle_compensate_multiple < 1)
      angle_compensate_multiple = 1;
    max_distance = current_scan_mode.max_distance;
    ROS_INFO(
        "current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , "
        "angle_compensate: %d",
        current_scan_mode.scan_mode, current_scan_mode.max_distance,
        (1000 / current_scan_mode.us_per_sample), angle_compensate_multiple);
  }
  else
  {
    ROS_ERROR("Can not start scan: %08x!", op_result);
  }

  ros::Time start_scan_time;
  ros::Time end_scan_time;
  double scan_duration;
  while (ros::ok())
  {
    rplidar_response_measurement_node_hq_t nodes[360 * 8];
    size_t count = _countof(nodes);

    start_scan_time = ros::Time::now();
    op_result = gDrv->grabScanDataHq(nodes, count);
    end_scan_time = ros::Time::now();
    scan_duration = (end_scan_time - start_scan_time).toSec();

    if (op_result == RESULT_OK)
    {
      op_result = gDrv->ascendScanData(nodes, count);
      float angle_min = DEG2RAD(0.0f);
      float angle_max = DEG2RAD(359.0f);
      if (op_result == RESULT_OK)
      {
        if (angle_compensate)
        {
          // const int angle_compensate_multiple = 1;
          const int angle_compensate_nodes_count =
              360 * angle_compensate_multiple;
          int angle_compensate_offset = 0;
          rplidar_response_measurement_node_hq_t
              angle_compensate_nodes[angle_compensate_nodes_count];
          memset(angle_compensate_nodes, 0,
                 angle_compensate_nodes_count *
                     sizeof(rplidar_response_measurement_node_hq_t));

          int i = 0, j = 0;
          for (; i < count; i++)
          {
            if (nodes[i].dist_mm_q2 != 0)
            {
              float angle = getAngle(nodes[i]);
              int angle_value = ( int )(angle * angle_compensate_multiple);
              if ((angle_value - angle_compensate_offset) < 0)
                angle_compensate_offset = angle_value;
              for (j = 0; j < angle_compensate_multiple; j++)
              {
                angle_compensate_nodes[angle_value - angle_compensate_offset +
                                       j] = nodes[i];
              }
            }
          }

          publish_scan(&scan_pub, angle_compensate_nodes,
                       angle_compensate_nodes_count, start_scan_time,
                       scan_duration, inverted, angle_min, angle_max,
                       max_distance, frame_id);
        }
        else
        {
          int start_node = 0, end_node = 0;
          int i = 0;
          // find the first valid node and last valid node
          while (nodes[i++].dist_mm_q2 == 0)
            ;
          start_node = i - 1;
          i = count - 1;
          while (nodes[i--].dist_mm_q2 == 0)
            ;
          end_node = i + 1;

          angle_min = DEG2RAD(getAngle(nodes[start_node]));
          angle_max = DEG2RAD(getAngle(nodes[end_node]));

          publish_scan(&scan_pub, &nodes[start_node], end_node - start_node + 1,
                       start_scan_time, scan_duration, inverted, angle_min,
                       angle_max, max_distance, frame_id);
        }
      }
      else if (op_result == RESULT_OPERATION_FAIL)
      {
        // All the data is invalid, just publish them
        float angle_min = DEG2RAD(0.0f);
        float angle_max = DEG2RAD(359.0f);

        publish_scan(&scan_pub, nodes, count, start_scan_time, scan_duration,
                     inverted, angle_min, angle_max, max_distance, frame_id);
      }
    }

    ros::spinOnce();
  }

  // Done with sending. Stop motor and driver. Ultimately remove the driver.
  gDrv->stop();
  gDrv->stopMotor();
  RPlidarDriver::DisposeDriver(gDrv);
  return 0;
}