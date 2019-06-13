
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

commands::Command::Command(const Command& other)
    : mType(other.mType),
      mChannel(other.mChannel),
      mRad(other.mRad),
      mSpeedFactor(other.mSpeedFactor),
      mTime(other.mTime)

{
}

commands::Command& commands::Command::operator=(const Command& other)
{
  if (this != &other)
  {
    mType = other.mType;
    mChannel = other.mChannel;
    mRad = other.mRad;
    mSpeedFactor = other.mSpeedFactor;
    mPwm = other.mPwm;
    mSpeed = other.mSpeed;
    mTime = other.mTime;
  }
  return *this;
}

bool commands::Command::operator==(const Command& other) const
{
  if (this == &other)
  {
    return true;
  }
  return mType == other.mType && mChannel == other.mChannel &&
         mRad == other.mRad && mSpeedFactor == other.mSpeedFactor;
}

bool commands::Command::operator!=(const Command& other) const
{
  return !(Command::operator==(other));
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
