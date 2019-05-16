
#include <sim_robot/Command.hpp>

commands::Command::Command(CommandType aType,
                           jointChannel_t aChannel,
                           jointPw_t aPwm,
                           jointVel_t aSpeed,
                           commandTime_t aTime)
    : type(aType), channel(aChannel), pwm(aPwm), speed(aSpeed), time(aTime)
{
}
commands::Command::Command(CommandType aType,
                           jointChannel_t aChannel,
                           jointRad_t aRad,
                           jointVel_t aSpeedFactor)
    : type(aType), channel(aChannel), rad(aRad), speedFactor(aSpeedFactor)
{
}
commands::CommandType commands::Command::getType() const
{
  return type;
}

jointChannel_t commands::Command::getChannel() const
{
  return channel;
}

jointPw_t commands::Command::getPwm() const
{
  return pwm;
}
jointRad_t commands::Command::getRad() const
{
  return rad;
}

jointVel_t commands::Command::getSpeed() const
{
  return speed;
}
jointVel_t commands::Command::getSpeedFactor() const
{
  return speedFactor;
}
commandTime_t commands::Command::getTime() const
{
  return time;
}
