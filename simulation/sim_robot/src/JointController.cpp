
#include <ros/ros.h>
#include <sim_robot/JointController.hpp>
#include <thread>

namespace gazebo
{
  bool equalsDouble(const double& a, const double& b)
  {
    uint16_t aPrecision = 100;
    auto precision = std::numeric_limits<double>::epsilon() * aPrecision;
    return precision >= std::abs(a - b);
  }

  JointController::JointController(physics::JointPtr& joint,
                                   const std::string& mName,
                                   jointChannel_t mChannel,
                                   jointPw_t mMinPw,
                                   jointPw_t mMaxPw,
                                   jointRad_t mMinRad,
                                   jointRad_t mMaxRad,
                                   jointVel_t mMaxVel)
      : joint(joint),
        mName(mName),
        mChannel(mChannel),
        mMinPw(mMinPw),
        mMaxPw(mMaxPw),
        mMinRad(mMinRad),
        mMaxRad(mMaxRad),
        mMaxVel(mMaxVel),
        mCurrentPos(0),
        mCurrentVel(0),
        mTargetPos(0),
        mStepSize(0)
  {
  }

  JointController::JointController(const JointController& other)
      : joint(other.joint),
        mName(other.mName),
        mChannel(other.mChannel),
        mMinPw(other.mMinPw),
        mMaxPw(other.mMaxPw),
        mMinRad(other.mMinRad),
        mMaxRad(other.mMaxRad),
        mMaxVel(other.mMaxVel),
        mCurrentPos(other.mCurrentPos),
        mCurrentVel(other.mCurrentVel),
        mTargetPos(other.mTargetPos),
        mStepSize(other.mStepSize)
  {
  }

  jointRad_t JointController::converseScaleToRad(double aScale,
                                                 double aMinScale,
                                                 double aMaxScale)
  {
    return mMinRad + (mMaxRad - mMinRad) *
                         ((aScale - aMinScale) / (aMaxScale - aMinScale));
  }

  JointController& JointController::operator=(const JointController& other)
  {
    if (this != &other)
    {
      joint = other.joint;
      mName = other.mName;
      mChannel = other.mChannel;
      mMinPw = other.mMinPw;
      mMaxPw = other.mMaxPw;
      mMinRad = other.mMinRad;
      mMaxRad = other.mMaxRad;
      mCurrentPos = other.mCurrentPos;
      mMaxVel = other.mMaxVel;
      mCurrentVel = other.mCurrentVel;
      mTargetPos = other.mTargetPos;
      mStepSize = other.mStepSize;
    }
    return *this;
  }

  bool JointController::operator==(const JointController& other) const
  {
    if (this == &other)
    {
      return true;
    }
    return joint == other.joint && mName == other.mName &&
           mChannel == other.mChannel && equalsDouble(mMinPw, other.mMinPw) &&
           equalsDouble(mMaxPw, other.mMaxPw) &&
           equalsDouble(mMinRad, other.mMinRad) &&
           equalsDouble(mMaxRad, other.mMaxRad) &&
           equalsDouble(mCurrentPos, other.mCurrentPos) &&
           equalsDouble(mMaxVel, other.mMaxVel) &&
           equalsDouble(mCurrentVel, other.mCurrentVel) &&
           equalsDouble(mTargetPos, other.mTargetPos) &&
           equalsDouble(mStepSize, other.mStepSize);
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
      joint->SetVelocity(0, mCurrentVel);
      joint->SetForce(0, 0);
      joint->SetPosition(0, mCurrentPos);
    }
  }

  bool JointController::moveTheta(jointRad_t aRad,
                                  jointVel_t aSpeedFactor,
                                  commandTime_t aTime,
                                  double aUpdateRate)
  {

    mTargetPos = aRad;

    if (equalsDouble(mTargetPos, mCurrentPos))
    {
      return false;
    }

    auto distance = std::abs(mTargetPos - mCurrentPos);

    if (aSpeedFactor <= 0 && aTime == 0)
    {
      aSpeedFactor = 0.1;
    }
    if (aSpeedFactor > 0)
    {
      mCurrentVel = M_PI_2 * aSpeedFactor;
    }
    else
    {
      mCurrentVel = 0;
    }
    if (aTime > 0)
    {
      auto time_vel = distance * (1000.0 / aTime);
      mCurrentVel = time_vel > mCurrentVel ? time_vel : mCurrentVel;
    }

    if (mCurrentVel > mMaxVel)
    {
      ROS_WARN("Joint [%s] speed to high! Using max [%f] instead of [%f]",
               mName.c_str(), mMaxVel, getCurrentVel());
      mCurrentVel = mMaxVel;
    }

    mStepSize = ((distance / aUpdateRate) * (mCurrentVel / distance));

    ROS_DEBUG("Joint [%s] moving to [%f] in [%f] rad per sec", mName.c_str(),
              mTargetPos, mCurrentVel);

    return true;
  }
  bool JointController::move(jointPw_t aPw,
                             jointVel_t aSpeed,
                             commandTime_t aTime,
                             double aUpdateRate)
  {
    if (inRange(aPw))
    {
      mTargetPos = convertPw2Radians(aPw);

      if (equalsDouble(mTargetPos, mCurrentPos))
      {
        return false;
      }

      auto distance = std::abs(mTargetPos - mCurrentPos);

      if (aSpeed <= 0 && aTime == 0)
      {
        aSpeed = 1000;
      }
      if (aSpeed > 0)
      {
        mCurrentVel = M_PI_2 / (1000 / aSpeed);
      }
      else
      {
        mCurrentVel = 0;
      }
      if (aTime > 0)
      {
        auto time_vel = distance * (1000.0 / aTime);
        mCurrentVel = time_vel > mCurrentVel ? time_vel : mCurrentVel;
      }

      if (mCurrentVel > mMaxVel)
      {
        ROS_WARN("Joint [%s] speed to high! Using max [%f] instead of [%f]",
                 mName.c_str(), mMaxVel, mCurrentVel);
        mCurrentVel = mMaxVel;
      }

      mStepSize = ((distance / aUpdateRate) * (mCurrentVel / distance));

      ROS_DEBUG("Joint [%s] moving to [%f] in [%f] rad per sec", mName.c_str(),
                mTargetPos, mCurrentVel);

      return true;
    }
    else
    {
      ROS_WARN("PW [%f] not in range for joint [%s] (min [%f] - max [%f])", aPw,
               mName.c_str(), mMinPw, mMaxPw);
      return false;
    }
  }

  void JointController::stop()
  {
    setCurrentVel(0);
    mTargetPos = mCurrentPos;
  }

  bool JointController::inRange(jointPw_t aPw) const
  {
    return (aPw >= mMinPw && aPw <= mMaxPw);
  }

  jointRad_t JointController::convertPw2Radians(jointPw_t aPw) const
  {
    return (aPw - mMinPw) * (mMaxRad - mMinRad) / (mMaxPw - mMinPw) + mMinRad;
  }

  void JointController::run()
  {
    if (equalsDouble(getTargetPos(), getCurrentPos()))
    {
      // Nowhere to run to
      return;
    }
    if (std::abs(getTargetPos() - getCurrentPos()) < mStepSize)
    {
      // Next step is the last one
      setCurrentPos(getTargetPos());
    }
    else
    {
      mTargetPos > mCurrentPos ? mCurrentPos += mStepSize
                               : mCurrentPos -= mStepSize;
    }
  }

  jointRad_t JointController::getTargetPos() const
  {
    return mTargetPos;
  }

  jointRad_t JointController::getCurrentPos() const
  {
    return mCurrentPos;
  }
  double JointController::getCurrentForce() const
  {
    return mCurrentForce;
  }

  void JointController::setCurrentPos(jointRad_t aCurrentPos)
  {
    mCurrentPos = aCurrentPos;
  }

  jointVel_t JointController::getCurrentVel() const
  {
    return mCurrentVel;
  }
  void JointController::setCurrentVel(jointVel_t aCurrentVel)
  {
    mCurrentVel = aCurrentVel;
  }
} // namespace gazebo