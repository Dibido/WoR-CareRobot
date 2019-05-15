#ifndef KINEMATICS_JOINT_HPP
#define KINEMATICS_JOINT_HPP

#include <string>
namespace kinematics
{

  /**
   * @brief Enum to denote the Type of Joint a Link has
   *
   */
  enum class Joint
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

  class JointHelper
  {
      public:
    static const std::string toString(Joint joint)
    {
      switch (joint)
      {
      case Joint::PRISMATIC:
        return "prismatic";
      case Joint::REVOLUTE:
        return "revolute";
      case Joint::STATIC:
        return "static";
      }
      return "error";
    }
  };

} // namespace kinematics
#endif // KINEMATICS_JOINT_HPP
