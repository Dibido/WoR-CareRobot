
#ifndef PROJECT_COMMAND_HPP
#define PROJECT_COMMAND_HPP

#include "types.hpp"
#include <map>

namespace commands
{
  /**
   * @brief data container with different states
   * @param UNDEFINED startup state
   * @param MOVE moving state of the arm
   * @param STOP stop state when recieving a stop message
   */
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
    /**
     * @brief Construct a command data container
     * @param aType : message type
     * @param aChannel : Index / Channel of the joint
     * @param aPwm :  pulse width to move joint to
     * @param aSpeed : Maximum velocity in radians per second (ex. M_PI_2 will
     * result in 1 second per 90 degrees)
     * @param aTime : duration of the movement
     */
    Command(eCommandType aType,
            jointChannel_t aChannel,
            jointPw_t aPwm,
            jointVel_t aSpeed,
            commandTime_t aTime);

    /**
     * @brief Construct a command data container
     * @param aType : message type
     * @param aChannel : Index / Channel of the joint
     * @param aRad :  radian to move joint to
     * @param aSpeedFactor : Maximum velocity in radians per second (ex. 1.0
     * will result in 180(or 150 depending on joint channel) degrees movement
     * per 1 second. )
     */
    Command(eCommandType aType,
            jointChannel_t aChannel,
            jointRad_t aRad,
            jointVel_t aSpeedFactor);
    Command() = delete;
    virtual ~Command() = default;
    Command(const Command& other);
    Command& operator=(const Command& other);
    bool operator==(const Command& other) const;
    bool operator!=(const Command& other) const;

    eCommandType getType() const;
    jointChannel_t getChannel() const;
    jointPw_t getPwm() const;
    jointRad_t getRad() const;
    jointVel_t getSpeed() const;
    jointVel_t getSpeedFactor() const;
    commandTime_t getTime() const;

      private:
    eCommandType mType = eCommandType::UNDEFINED;
    jointChannel_t mChannel;
    jointPw_t mPwm = 0;
    jointRad_t mRad = 0;
    jointVel_t mSpeed = 0;
    jointVel_t mSpeedFactor = 0;
    commandTime_t mTime = 0;
  };

} // namespace commands

#endif // PROJECT_COMMAND_HPP
