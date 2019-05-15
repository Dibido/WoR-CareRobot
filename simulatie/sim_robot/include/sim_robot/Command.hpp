
#ifndef PROJECT_COMMAND_HPP
#define PROJECT_COMMAND_HPP

#include <map>
#include "types.hpp"

namespace commands
{
typedef enum CommandType
{
  UNDEFINED = -1,
  MOVE,
  STOP
} Type;

/**
 * Simple class to store values from an incoming command
 */
class Command
{
public:
  Command(CommandType aType, jointChannel_t aChannel, jointPw_t aPwm, jointVel_t aSpeed, commandTime_t aTime);
  Command() = delete;
  virtual ~Command() = default;

  CommandType getType() const;
  jointChannel_t getChannel() const;
  jointPw_t getPwm() const;
  jointVel_t getSpeed() const;
  commandTime_t getTime() const;

private:
  CommandType type = UNDEFINED;
  jointChannel_t channel;
  jointPw_t pwm;
  jointVel_t speed;
  commandTime_t time;
};

}  // namespace commands

#endif  // PROJECT_COMMAND_HPP
