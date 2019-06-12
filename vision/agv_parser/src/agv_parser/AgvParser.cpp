#include "agv_parser/AgvParser.hpp"

void agv_parser::AgvParser::Run()
{
  // Open serial
  boost::system::error_code lBoostError;
  mSerial->open(mSerialPort, lBoostError);
  std::cout << "AvgParser started." << std::endl;
  while (true)
  {
    // Get message
    std::string lRecievedMessage = readLine();
    std::cout << "Recieved message : \"" << lRecievedMessage << "\""
              << std::endl;
    if (lRecievedMessage.length() > 5)
    {
      // Parse message
      AgvSpeed lAgvSpeed;
      lAgvSpeed.mAgvSpeed = parseRecievedMessage(lRecievedMessage);
      parseAgvSpeed(lAgvSpeed);
      std::cout << "Handled message : " << lRecievedMessage << std::endl;
    }
    else
    {
      std::cout << "Message is too short" << std::endl;
    }
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
  // Example command = "#S#0.23131\n"
  // Strip the #S part from the message
  std::string lStrippedHashString = aRecievedMessage.substr(
      aRecievedMessage.find('#') + 1, aRecievedMessage.size());
  // Read the value after the #
  std::string lSpeedString = lStrippedHashString.substr(
      aRecievedMessage.find('#') + 2, aRecievedMessage.size());
  // Example command = "#S#0.23231423\n"
  // Check the length of the message.
  if (aRecievedMessage.length() <=
      cSmallestLegalCommandLength) // Should at least contain "#S#0"
  {
    throw std::invalid_argument("The message length is invalid");
  }
  // Example command = "#S#0.23231423\n"
  // Check the format
  // Check the #S
  std::string lCommandPrefix = aRecievedMessage.substr(0, 2);
  if (lCommandPrefix != agv_parser::cCommandHeader &&
      lCommandPrefix != agv_parser::cIntervalHeader)
  {
    // Strip the #I part from the message
    std::string lStrippedHashString = aRecievedMessage.substr(
        aRecievedMessage.find(agv_parser::cCommandDelimiter) + 1,
        aRecievedMessage.size());
    // Read the value after the #
    std::string lIntervalString = lStrippedHashString.substr(
        aRecievedMessage.find(agv_parser::cCommandDelimiter) + 2,
        aRecievedMessage.size());
    float lAgvInterval = static_cast<float>(atof(lIntervalString.c_str()));
    ROS_INFO("Interval : %f", lAgvInterval);
    return 0;
  }
  // Check for #I, interval message
  if (lCommandPrefix == agv_parser::cIntervalHeader)
  {
    // Strip the #I part from the message
    std::string lStrippedHashString = aRecievedMessage.substr(
        aRecievedMessage.find(agv_parser::cCommandDelimiter) + 1,
        aRecievedMessage.size());
    // Read the value after the #
    std::string lIntervalString = lStrippedHashString.substr(
        aRecievedMessage.find(agv_parser::cCommandDelimiter) + 2,
        aRecievedMessage.size());
    float lAgvInterval = static_cast<float>(atof(lIntervalString.c_str()));
    ROS_INFO("Interval : %f", lAgvInterval);
    return 0;
  }
  else if (lCommandPrefix == agv_parser::cCommandHeader)
  {
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
  else
  {
    throw std::invalid_argument("The message format is invalid");
  }
}

void agv_parser::AgvParser::parseAgvSpeed(const AgvSpeed& aAgvSpeed)
{
  sensor_interfaces::AGVSpeed lAgvMessage;
  lAgvMessage.speed = aAgvSpeed.mAgvSpeed;
  // Publish to ROS topic
  mAgvPublisher.publish(lAgvMessage);
}