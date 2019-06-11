#ifndef AGV_PARSER_HPP
#define AGV_PARSER_HPP

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>

#include "AgvSpeed.hpp"
#include "IAgvSpeedProvider.hpp"
#include "sensor_interfaces/AGVSpeed.h"
#include <ros/ros.h>

#include <iostream>
#include <string>

<<<<<<< Updated upstream
=======
// Constants for the agv_parser
namespace agv_parser
{
  const std::string cCommandHeader = "#S";
  const std::string cIntervalHeader = "#I";
  const char cCommandDelimiter = '#';
  const unsigned int cSmallestLegalCommandLength = 4;
  const char cReturnChar = '\r';
  const char cNewlineChar = '\n';
  const std::string cAgvSpeedTopic = "/sensor/agv";
  const unsigned int cBaudrate = 115200;
} // namespace agv_parser

>>>>>>> Stashed changes
namespace agv_parser
{
  /**
   * @brief Class that handles the AGV data and sends it to a ROS topic.
   */
  class AgvParser : public IAgvSpeedProvider
  {
      public:
    AgvParser(std::string aPort);
    virtual ~AgvParser();

    /**
     * @brief - Handles the serial communication and sends a ROS message.
     */
    void Run();

      private:
    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine();

    /**
     * @brief - Parses the recieved message to a float
     */
    float parseRecievedMessage(std::string aRecievedMessage);

    /**
     * @brief Implement the AGVSpeed function from the IAgvSpeed interface.
     * @param aAgvSpeed - The speed recieved from the AGV gateway.
     */
    virtual void parseAgvSpeed(const AgvSpeed& aAgvSpeed);

    // Serial variables
    std::string mSerialPort;
    boost::asio::io_service mIoService;
    boost::asio::serial_port mSerial;

    // Ros variables
    ros::NodeHandlePtr mRosNode;
    ros::Publisher mAgvPublisher;
  };
} // namespace agv_parser

#endif /* AGV_PARSER_HPP*/