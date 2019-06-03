#include <regex>
#include <ros/ros.h>
#include <string>

#include <sim_robot/CommandParser.hpp>
#include <sim_robot/types.hpp>

namespace commands
{
  void CommandParser::parseCommand(const std::string& command,
                                   std::vector<Command>& out)
  {
    std::vector<std::string> lineContainer = {};
    splitString(command, lineContainer);

    std::regex TimeRegex(".*T([0-9]+)$");
    std::smatch matches;

    for (std::string& s : lineContainer)
    {
      std::string commandString;
      std::regex space_reg("[[:space:]]+");
      commandString = std::regex_replace(s, space_reg, "");

      if (std::regex_match(commandString, matches, TimeRegex))
      {
        createCommand(commandString, out,
                      static_cast<commandTime_t>(std::stoul(matches[1].str())));
      }
      else
      {
        createCommand(commandString, out);
      }
    }
  }
  void CommandParser::parseControl(const std::vector<double>& commandTheta,
                                   jointVel_t speedFactor,
                                   std::vector<Command>& thetaOut)
  {

    if (commandTheta.empty())
    { // the match on commandTheta is the speed (if it exists) else this field
      // is
      // empty
      speedFactor = 0;
    }
    createControlCommand(commandTheta, speedFactor, thetaOut);
  }
  void CommandParser::parseStop(const bool& stop, std::vector<Command>& out)
  {

    createStopCommand(stop, out);
  }

  void CommandParser::createControlCommand(
      const std::vector<double>& commandTheta,
      jointVel_t speedFactor,
      std::vector<commands::Command>& container)
  {

    for (uint16_t i = 0; i < commandTheta.size(); ++i)
    {

      Command command(eCommandType::MOVE,                       // type
                      static_cast<jointChannel_t>(i),           // channel
                      static_cast<jointRad_t>(commandTheta[i]), // rad
                      speedFactor);                             // speedFactor
      container.push_back(command);
    }
  }

  void CommandParser::createStopCommand(
      const bool& stop,
      std::vector<commands::Command>& container)
  {
    if (stop)
    {
      for (uint16_t i = 0; i < 7; ++i)
      {
        Command command(eCommandType::STOP,             // type
                        static_cast<jointChannel_t>(i), // channel
                        static_cast<jointRad_t>(0),     // rad
                        0);                             // speedFactor
        container.push_back(command);
      }
    }
  }

  bool CommandParser::isMoveCommand(const std::string& line)
  {
    std::string commandString;
    std::regex space_reg("[[:space:]]+");
    commandString = std::regex_replace(line, space_reg, "");

    std::regex CommandRegex(
        "(?:#([0-9]+)P(?!-?[0-9]+(?:\\.[0-9]+)\\.)(-?[0-9]+(?:\\.[0-9]+)?)(?:S("
        "[0-9]+))?)+(?:T(?:[0-"
        "9]+))?");
    return std::regex_match(commandString, CommandRegex);
  }

  bool CommandParser::isStopCommand(const std::string& line)
  {
    std::string commandString;
    std::regex space_reg("[[:space:]]+");
    commandString = std::regex_replace(line, space_reg, "");

    std::regex CommandRegex("STOP");
    return std::regex_match(commandString, CommandRegex);
  }

  void CommandParser::splitString(const std::string& input,
                                  std::vector<std::string>& container,
                                  char delim /*='\r'*/)
  {
    std::stringstream stringStream(input);
    std::string token;
    while (std::getline(stringStream, token, delim))
    {
      if (!token.empty())
      {
        container.push_back(token);
      }
    }
  }

  void CommandParser::createCommand(const std::string& line,
                                    std::vector<Command>& container,
                                    commandTime_t time /*=0*/)
  {
    if (isMoveCommand(line))
    {
      createCommandMove(line, container, time);
    }
    else if (isStopCommand(line))
    {
      createCommandStop(line, container, time);
    }
    else
    {
      ROS_WARN("Command not recognized: %s", line.c_str());
    }
  }

  void CommandParser::createCommandMove(const std::string& line,
                                        std::vector<Command>& container,
                                        commandTime_t time /*=0*/)
  {
    std::regex CommandRegex("#([0-9]+)P(-?[0-9]+(?:\\.[0-9]+)?)(?:S([0-9]+))?");
    std::smatch matches;

    std::string::const_iterator searchStart(line.cbegin());

    while (std::regex_search(searchStart, line.cend(), matches, CommandRegex))
    {
      jointVel_t tempSpeed = 0;
      if (!matches[3].str().empty())
      { // the match on pos "3" is the speed (if it exists) else this field is
        // empty
        tempSpeed = static_cast<jointVel_t>(std::stod(matches[3].str()));
      }
      Command command(
          eCommandType::MOVE,                                        // type
          static_cast<jointChannel_t>(std::stoul(matches[1].str())), // channel
          static_cast<jointPw_t>(std::stod(matches[2].str())),       // pwm
          tempSpeed,                                                 // speed
          time                                                       // time
      );

      container.push_back(command);

      searchStart +=
          matches.position() + matches.length(); // iterate over the string
    }
  }

  void CommandParser::createCommandStop(const std::string& line,
                                        std::vector<Command>& container,
                                        commandTime_t time /*=0*/)
  {
    if (isStopCommand(line))
    {
      for (unsigned int i = 0; i < 7; ++i)
      {
        Command command(eCommandType::STOP,             // type
                        static_cast<jointChannel_t>(i), // channel
                        0,                              // pwm
                        0,                              // speed
                        time                            // time
        );
        container.push_back(command);
      }
    }
  }

} // namespace commands
