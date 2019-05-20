
#ifndef PROJECT_COMMAND_HPP
#define PROJECT_COMMAND_HPP

#include "types.hpp"
#include <map>

namespace commands
{
   enum class eCommandType
  {
    UNDEFINED = -1,
    MOVE,
    STOP
  };

  /**
   *  class to store values from an incoming command
   */
  class Command
  {
      public:
    Command(eCommandType aType,
            jointChannel_t aChannel,
            jointPw_t aPwm,
            jointVel_t aSpeed,
            commandTime_t aTime);
    Command(eCommandType aType,
            jointChannel_t aChannel,
            jointRad_t aRad,
            jointVel_t aSpeedFactor);
    Command() = delete;
    virtual ~Command() = default;

    eCommandType getType() const;
    jointChannel_t getChannel() const;
    jointPw_t getPwm() const;
    jointRad_t getRad() const;
    jointVel_t getSpeed() const;
    jointVel_t getSpeedFactor() const;
    commandTime_t getTime() const;

      private:
    eCommandType type = eCommandType::UNDEFINED;
    jointChannel_t mChannel;
    jointPw_t mPwm=0;
    jointRad_t mRad=0;
    jointVel_t mSpeed=0;
    jointVel_t mSpeedFactor=0;
    commandTime_t mTime=0;
  };

} // namespace commands

#endif // PROJECT_COMMAND_HPP
