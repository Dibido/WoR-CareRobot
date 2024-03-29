#include "agv_parser/AgvParser.hpp"

void agv_parser::AgvParser::run()
{
  // Open serial
  boost::system::error_code lBoostError;
  mSerial->open(mSerialPort, lBoostError);
  ROS_INFO("AvgParser started.");
  while (true)
  {
    // Get message
    std::string lRecievedMessage = readLine();
    // Parse message
    AgvSpeed lAgvSpeed;
    lAgvSpeed.mAgvSpeed = parseRecievedMessage(lRecievedMessage);
    parseAgvSpeed(lAgvSpeed);
    ROS_INFO("Handled message : %s", lRecievedMessage.c_str());
  }
}

agv_parser::AgvParser::AgvParser(std::string aPort)
    : mIoService(),
      mSerial(boost::make_shared<boost::asio::serial_port>(mIoService, aPort))
{
  // Set up serial
  mSerialPort = aPort;
  mSerial->set_option(
      boost::asio::serial_port_base::baud_rate(agv_parser::cBaudrate));
  // Set up ROS
  // Create a ros NodeHandle
  mRosNode = std::make_unique<ros::NodeHandle>();
  // Advertise ROS node
  mAgvPublisher = mRosNode->advertise<sensor_interfaces::AGVSpeed>(
      agv_parser::cAgvSpeedTopic, 1);
}

agv_parser::AgvParser::AgvParser()
    : mIoService(),
      mSerial(boost::make_shared<boost::asio::serial_port>(mIoService))
{
  // Create a ros NodeHandle
  mRosNode = std::make_unique<ros::NodeHandle>();
  // Advertise ROS node
  mAgvPublisher = mRosNode->advertise<sensor_interfaces::AGVSpeed>(
      agv_parser::cAgvSpeedTopic, 1);
}

agv_parser::AgvParser::~AgvParser()
{
  // Tear down serial
}

std::string agv_parser::AgvParser::readLine()
{
  // Reading data char by char, code is optimized for simplicity, not speed
  char lCurrentChar;
  std::string lResultString;
  for (;;)
  {
    boost::asio::read(*mSerial, boost::asio::buffer(&lCurrentChar, 1));
    switch (lCurrentChar)
    {
    case agv_parser::cReturnChar:
      break;
    case agv_parser::cNewlineChar:
      return lResultString;
    default:
      lResultString += lCurrentChar;
    }
  }
}

float agv_parser::AgvParser::parseRecievedMessage(std::string aRecievedMessage)
{
  // Parse the command
  // Example command = "#S#0.23231423\n"
  // Check the length of the message.
  if (aRecievedMessage.length() <=
      cSmallestLegalCommandLength) // Should at least contain "#S#0"
  {
    throw std::invalid_argument("The message length is invalid");
  }
  // Check the format
  // Check the #S
  std::string lCommandPrefix = aRecievedMessage.substr(0, 2);
  if (lCommandPrefix != agv_parser::cCommandHeader)
  {
    throw std::invalid_argument("The message format is invalid");
  }
  // Strip the #S part from the message
  std::string lStrippedHashString = aRecievedMessage.substr(
      aRecievedMessage.find(agv_parser::cCommandDelimiter) + 1,
      aRecievedMessage.size());
  // Read the value after the #
  std::string lSpeedString = lStrippedHashString.substr(
      aRecievedMessage.find(agv_parser::cCommandDelimiter) + 2,
      aRecievedMessage.size());
  float lAgvSpeed = static_cast<float>(atof(lSpeedString.c_str()));
  // Check the value
  if (lAgvSpeed <= 0)
  {
    throw std::invalid_argument("The speed is invalid");
  }
  return lAgvSpeed;
}

void agv_parser::AgvParser::parseAgvSpeed(const AgvSpeed& aAgvSpeed)
{
  sensor_interfaces::AGVSpeed lAgvMessage;
  lAgvMessage.speed = aAgvSpeed.mAgvSpeed;
  // Publish to ROS topic
  mAgvPublisher.publish(lAgvMessage);
}