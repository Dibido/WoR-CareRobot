#ifndef PROJECT_COMMANDPARSER_HPP
#define PROJECT_COMMANDPARSER_HPP

#include "Command.hpp"
#include <string>
#include <vector>

namespace commands
{
  /**
   * Parser class to parse commands with the lynxmotion protocol
   */
  class CommandParser
  {
      public:
    void parseCommand(const std::string& command,
                      std::vector<commands::Command>& out);

    /**
     * Parser class to parse commands with the custom made protocol
     */
    void parseCommandTheta(const std::vector<double>& commandTheta,
                           jointVel_t speedFactor,
                           std::vector<commands::Command>& thetaOut);

    /**
     * \brief this function is used to check if a string complies with the
     * protocol of a move command
     */
    bool isMoveCommand(const std::string& line);
    /**
     * \brief this function is used to check if a string complies with the
     * protocol of a stop command
     */
    bool isStopCommand(const std::string& line);
    /**
     * \brief create a list of strings from 1 string spliting this string by the
     * delimiter
     * @param the input string that is about to get split
     * @param the vector where the results get pushed to
     * @param the delimiter that is used to split the string, default the '\r'
     * character
     */
    void splitString(const std::string& input,
                     std::vector<std::string>& container,
                     char delim = '\r');
    /**
     * \brief fill the command container with the data from the command line no
     * matter what type of command it is
     * @param the data for the container in the SSC32U protocol
     *  * @param container that gets filled with the data
     */

      private:
    void createCommand(const std::string& line,
                       std::vector<Command>& container,
                       uint32_t time = 0);
    /**
     * \brief fill the command container with the data from the /robot_command topic
     * @param the data for the container in the custom made protocol
     *  * @param container that gets filled with the data
     */
    void createCommandTheta(const std::vector<double>& commandTheta,
                            jointVel_t speedFactor,
                            std::vector<commands::Command>& thetaOut);
    /**
     * \brief fill the command container with the data from the command line
     * for a move command only call with line with removed spaces
     * @param the data for the container in the SSC32U protocol
     *  * @param container that gets filled with the data
     */
    void createCommandMove(const std::string& line,
                           std::vector<Command>& container,
                           uint32_t time = 0);
    /**
     * \brief fill the command container with the data from the command line for
     * a stop command only call with line with removed spaces
     * @param the data for the container in the SSC32U protocol
     * @param container that gets filled with the data
     */
    void createCommandStop(const std::string& line,
                           std::vector<Command>& container,
                           uint32_t time = 0);
  };

} // namespace commands

#endif // PROJECT_COMMANDPARSER_HPP
