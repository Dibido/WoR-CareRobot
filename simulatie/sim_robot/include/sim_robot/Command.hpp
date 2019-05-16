
#ifndef PROJECT_COMMAND_HPP
#define PROJECT_COMMAND_HPP

#include "types.hpp"
#include <map>

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
    Command(CommandType aType,
            jointChannel_t aChannel,
            jointPw_t aPwm,
            jointVel_t aSpeed,
            commandTime_t aTime);
    Command(CommandType aType,
            jointChannel_t aChannel,
            jointRad_t aRad,
            jointVel_t aSpeedFactor);
    Command() = delete;
    virtual ~Command() = default;

    CommandType getType() const;
    jointChannel_t getChannel() const;
    jointPw_t getPwm() const;
    jointRad_t getRad() const;
    jointVel_t getSpeed() const;
    jointVel_t getSpeedFactor() const;
    commandTime_t getTime() const;

      private:
    CommandType type = UNDEFINED;
    jointChannel_t channel;
    jointPw_t pwm;
    jointRad_t rad;
    jointVel_t speed;
    jointVel_t speedFactor;
    commandTime_t time;
  };

} // namespace commands

#endif // PROJECT_COMMAND_HPP
