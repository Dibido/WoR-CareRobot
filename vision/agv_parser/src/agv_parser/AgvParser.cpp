#include "agv_parser/AgvParser.hpp"

void agv_parser::AgvParser::run()
{
  // Open serial
  boost::system::error_code lBoostError;
  mSerial.open(mSerialPort, lBoostError);
  std::cout << "AvgParser started." << std::endl;
  while (true)
  {
    // Get message
    std::string lRecievedMessage = readLine();
    // Parse message
    AgvSpeed lAgvSpeed;
    lAgvSpeed.mAgvSpeed = parseRecievedMessage(lRecievedMessage);
    parseAgvSpeed(lAgvSpeed);
    std::cout << "Handled message : " << lRecievedMessage << std::endl;
  }
}

agv_parser::AgvParser::AgvParser(std::string aPort)
    : mIoService(), mSerial(mIoService, aPort)
{
  // Set up serial
  mSerialPort = aPort;
  mSerial.set_option(boost::asio::serial_port_base::baud_rate(9600));
  // Set up ROS
  // Create a ros NodeHandle
  mRosNode = std::make_unique<ros::NodeHandle>();
  // Advertise ROS node
  mAgvPublisher =
      mRosNode->advertise<sensor_interfaces::AGVSpeed>("/sensor/agv", 1);
}

agv_parser::AgvParser::~AgvParser()
{
  // Tear down serial
}

std::string agv_parser::AgvParser::readLine()
{
  // Reading data char by char, code is optimized for simplicity, not speed
  char c;
  std::string result;
  for (;;)
  {
    boost::asio::read(mSerial, boost::asio::buffer(&c, 1));
    switch (c)
    {
    case '\r':
      break;
    case '\n':
      return result;
    default:
      result += c;
    }
  }
}

float agv_parser::AgvParser::parseRecievedMessage(std::string aRecievedMessage)
{
  // Parse the command
  // Example command = "#S#0.23231\n"
  // Check the length of the message.
  unsigned int lSmallestLegalCommandLength = 4;
  if (aRecievedMessage.length() <=
      lSmallestLegalCommandLength) // Should at least contain "#S#0"
  {
    throw std::invalid_argument("The message length is invalid");
  }
  // Check the format
  // Check the #S
  std::string lCommandPrefix = aRecievedMessage.substr(0, 2);
  if (lCommandPrefix != "#S")
  {
    throw std::invalid_argument("The message format is invalid");
  }
  // Strip the #S part from the message
  std::string lStrippedHashString = aRecievedMessage.substr(
      aRecievedMessage.find('#') + 1, aRecievedMessage.size());
  // Read the value after the #
  std::string lSpeedString = lStrippedHashString.substr(
      aRecievedMessage.find('#') + 2, aRecievedMessage.size());
  float lAgvSpeed = static_cast<float>(atof(lSpeedString.c_str()));
  // Check the value
  if (lAgvSpeed < 0 || lAgvSpeed == 0)
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