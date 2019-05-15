
#include <sim_robot/Command.hpp>

commands::Command::Command(CommandType aType, jointChannel_t aChannel, jointPw_t aPwm, jointVel_t aSpeed,
                           commandTime_t aTime)
  : type(aType), channel(aChannel), pwm(aPwm), speed(aSpeed), time(aTime)
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

jointVel_t commands::Command::getSpeed() const
{
  return speed;
}

commandTime_t commands::Command::getTime() const
{
  return time;
}
