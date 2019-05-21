
#include <sim_robot/Command.hpp>

commands::Command::Command(eCommandType aType,
                           jointChannel_t aChannel,
                           jointPw_t aPwm,
                           jointVel_t aSpeed,
                           commandTime_t aTime)
    : mType(aType), mChannel(aChannel), mPwm(aPwm), mSpeed(aSpeed), mTime(aTime)
{
}
commands::Command::Command(eCommandType aType,
                           jointChannel_t aChannel,
                           jointRad_t aRad,
                           jointVel_t aSpeedFactor)
    : mType(aType), mChannel(aChannel), mRad(aRad), mSpeedFactor(aSpeedFactor)
{
}
commands::eCommandType commands::Command::getType() const
{
  return mType;
}

jointChannel_t commands::Command::getChannel() const
{
  return mChannel;
}

jointPw_t commands::Command::getPwm() const
{
  return mPwm;
}
jointRad_t commands::Command::getRad() const
{
  return mRad;
}
jointVel_t commands::Command::getSpeed() const
{
  return mSpeed;
}
jointVel_t commands::Command::getSpeedFactor() const
{
  return mSpeedFactor;
}
commandTime_t commands::Command::getTime() const
{
  return mTime;
}
