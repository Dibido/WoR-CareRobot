#ifndef KINEMATICS_JOINT_HPP
#define KINEMATICS_JOINT_HPP

#include <string>
namespace kinematics
{

  /**
   * @brief Enum to denote the Type of Joint a Link has
   *
   */
  enum class eJoint
  {
    /**
     * @brief The Link increases its D value and keeps its Theta
     * value static
     */
    PRISMATIC,
    /**
     * @brief The Link rotates around the Z-axis, increases
     * Theta and keeps D static
     */
    REVOLUTE,
    /**
     * @brief The Link does not change, both Theta and D are
     * static
     */
    STATIC,
  };

  /**
   * @brief Helper to convert eJoint to string
   * 
   */
  class JointHelper
  {
      public:
      /**
       * @brief Convert eJoint to a string value
       * 
       * @param lJoint 
       * @return const std::string 
       */
    static const std::string toString(eJoint lJoint)
    {
      switch (lJoint)
      {
      case eJoint::PRISMATIC:
        return "prismatic";
      case eJoint::REVOLUTE:
        return "revolute";
      case eJoint::STATIC:
        return "static";
      }
      return "error";
    }
  };

} // namespace kinematics
#endif // KINEMATICS_JOINT_HPP
