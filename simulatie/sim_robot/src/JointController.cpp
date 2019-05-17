
#include <sim_robot/JointController.hpp>
#include <ros/ros.h>
#include <thread>

namespace gazebo
{
bool equalsDouble(const double& a, const double& b, uint16_t aPrecision = 100)
{
  auto precision = std::numeric_limits<double>::epsilon() * aPrecision;
  return precision >= std::abs(a - b);
}

JointController::JointController(physics::JointPtr& joint, const std::string& name, jointChannel_t channel,
                                 jointPw_t min_pw, jointPw_t max_pw, jointRad_t min_rad, jointRad_t max_rad,
                                 jointVel_t max_vel)
  : joint(joint)
  , name(name)
  , channel(channel)
  , min_pw(min_pw)
  , max_pw(max_pw)
  , min_rad(min_rad)
  , max_rad(max_rad)
  , max_vel(max_vel)
  , current_pos(0)
  , current_vel(0)
  , target_pos(0)
  , step_size(0)
{
}

JointController::JointController(const JointController& other)
  : joint(other.joint)
  , name(other.name)
  , channel(other.channel)
  , min_pw(other.min_pw)
  , max_pw(other.max_pw)
  , min_rad(other.min_rad)
  , max_rad(other.max_rad)
  , max_vel(other.max_vel)
  , current_pos(other.current_pos)
  , current_vel(other.current_vel)
  , target_pos(other.target_pos)
  , step_size(other.step_size)
{
}

JointController& JointController::operator=(const JointController& other)
{
  if (this != &other)
  {
    joint = other.joint;
    name = other.name;
    channel = other.channel;
    min_pw = other.min_pw;
    max_pw = other.max_pw;
    min_rad = other.min_rad;
    max_rad = other.max_rad;
    current_pos = other.current_pos;
    max_vel = other.max_vel;
    current_vel = other.current_vel;
    target_pos = other.target_pos;
    step_size = other.step_size;
  }
  return *this;
}

bool JointController::operator==(const JointController& other) const
{
  if (this == &other)
  {
    return true;
  }
  return joint == other.joint && name == other.name && channel == other.channel && equalsDouble(min_pw, other.min_pw) &&
         equalsDouble(max_pw, other.max_pw) && equalsDouble(min_rad, other.min_rad) &&
         equalsDouble(max_rad, other.max_rad) && equalsDouble(current_pos, other.current_pos) &&
         equalsDouble(max_vel, other.max_vel) && equalsDouble(current_vel, other.current_vel) &&
         equalsDouble(target_pos, other.target_pos) && equalsDouble(step_size, other.step_size);
}

bool JointController::operator!=(const JointController& other) const
{
  return !(JointController::operator==(other));
}

void JointController::update()
{
  run();
  if (joint)
  {
    joint->SetVelocity(0, current_vel);
    joint->SetForce(0, 0);
    joint->SetPosition(0, current_pos);
  }
}

bool JointController::moveTheta(jointRad_t rad, jointVel_t speed, commandTime_t time, double updateRate)
{
 
    target_pos = rad;

    if (equalsDouble(target_pos, current_pos))
    {
      return false;
    }

    auto distance = std::abs(target_pos - current_pos);

    if (speed <= 0 && time == 0)
    {
      speed = 1000;
    }
    if (speed > 0)
    {
      current_vel = M_PI_2 / (1000 / speed);
    }
    else
    {
      current_vel = 0;
    }
    if (time > 0)
    {
      auto time_vel = distance * (1000.0 / time);
      current_vel = time_vel > current_vel ? time_vel : current_vel;
    }

    if (current_vel > max_vel)
    {
      ROS_WARN("Joint [%s] speed to high! Using max [%f] instead of [%f]", name.c_str(), max_vel, current_vel);
      current_vel = max_vel;
    }

    step_size = ((distance / updateRate) * (current_vel / distance));

    ROS_DEBUG("Joint [%s] moving to [%f] in [%f] rad per sec", name.c_str(), target_pos, current_vel);

    return true;

}
bool JointController::move(jointPw_t pw, jointVel_t speed, commandTime_t time, double updateRate)
{
  if (inRange(pw))
  {
    target_pos = convertPw2Radians(pw);

    if (equalsDouble(target_pos, current_pos))
    {
      return false;
    }

    auto distance = std::abs(target_pos - current_pos);

    if (speed <= 0 && time == 0)
    {
      speed = 1000;
    }
    if (speed > 0)
    {
      current_vel = M_PI_2 / (1000 / speed);
    }
    else
    {
      current_vel = 0;
    }
    if (time > 0)
    {
      auto time_vel = distance * (1000.0 / time);
      current_vel = time_vel > current_vel ? time_vel : current_vel;
    }

    if (current_vel > max_vel)
    {
      ROS_WARN("Joint [%s] speed to high! Using max [%f] instead of [%f]", name.c_str(), max_vel, current_vel);
      current_vel = max_vel;
    }

    step_size = ((distance / updateRate) * (current_vel / distance));

    ROS_DEBUG("Joint [%s] moving to [%f] in [%f] rad per sec", name.c_str(), target_pos, current_vel);

    return true;
  }
  else
  {
    ROS_WARN("PW [%f] not in range for joint [%s] (min [%f] - max [%f])", pw, name.c_str(), min_pw, max_pw);
    return false;
  }
}

void JointController::stop()
{
  current_vel = 0;
  target_pos = current_pos;
}

bool JointController::inRange(jointPw_t pw) const
{
  return (pw >= min_pw && pw <= max_pw);
}

jointRad_t JointController::convertPw2Radians(jointPw_t pw) const
{
  return (pw - min_pw) * (max_rad - min_rad) / (max_pw - min_pw) + min_rad;
}

void JointController::run()
{
  if (equalsDouble(target_pos, current_pos))
  {
    // Nowhere to run to
    return;
  }
  if (std::abs(target_pos - current_pos) < step_size)
  {
    // Next step is the last one
    current_pos = target_pos;
  }
  else
  {
    target_pos > current_pos ? current_pos += step_size : current_pos -= step_size;
  }
}

jointRad_t JointController::getTargetPos() const
{
  return target_pos;
}

jointRad_t JointController::getCurrentPos() const
{
  return current_pos;
}

void JointController::setCurrentPos(jointRad_t aCurrentPos)
{
  current_pos = aCurrentPos;
}

jointVel_t JointController::getCurrentVel() const
{
  return current_vel;
}

}  // namespace gazebo