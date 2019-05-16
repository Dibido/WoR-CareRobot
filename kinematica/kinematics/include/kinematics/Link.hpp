#ifndef KINEMATICS_LINK_HPP
#define KINEMATICS_LINK_HPP

#include "kinematics/Joint.hpp"
#include "matrix/Matrix.hpp"

namespace kinematics
{

  /**
   * @brief Representation of a Link in a robotarm using Denavit
   * Hartenberg parameters
   * @see https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
   */
  class Link
  {
      public:
    /**
     * @brief Construct a new Link object
     *
     * @param lA_m Link length
     * @param lAlpha_rad Link twist
     * @param lD_m Link offset
     * @param lTheta_rad Joint angle
     * @param lType
     * @param lMinValue Radians if lType is REVOLUTE, meters if lType is PRISMATIC, unused if lType is STATIC
     * @param lMaxValue Radians if lType is REVOLUTE, meters if lType is PRISMATIC, unused if lType is STATIC
     */
    Link(double lA_m,
         double lAlpha_rad,
         double lD_m,
         double lTheta_rad,
         eJoint lType,
         double lMinValue = 0,
         double lMaxValue = 0);

    /**
     * @brief Construct a new Link object, sets the static value
     * to 0
     * @param lA_m Link Length
     * @param lAlpha_rad Link Twist
     * @param lConstant Value of the constant value (mD_m or
     * mTheta_rad)
     * @param lType Type of Joint (eJoint::PRISMATIC sets
     * mTheta_rad, eJoint::REVOLUTE sets mD_rad)
     * @param lMinValue
     * @param lMaxValue
     */
    Link(double lA_m,
         double lAlpha_rad,
         double lConstant,
         eJoint lType,
         double lMinValue,
         double lMaxValue);
    ~Link() = default;
    Link(const Link&) = default;
    Link& operator=(const Link&) = default;

    /**
     * @brief Get the Transformation Matrix for a given Link
     * @param lVariable Changing value for this Link (meters or radians)
     * @return Matrix<double, 4, 4> The TransformationMatrix
     * ~~~
     * [ r00 r11 r12 tx ]
     * [ r10 r21 r22 ty ]
     * [ r20 r31 r32 tz ]
     * [ 0   0   0   1  ]
     * ~~~
     */
    Matrix<double, 4, 4> transformationMatrix(double lVariable) const;

    double getA() const;
    double getAlpha() const;
    double getTheta() const;
    double getD() const;
    eJoint getType() const;

    /**
     * @brief Checks wether variable falls within lMinValue and lMaxValue
     * In case of a REVOLUTE joint, variable is first translated to fall between
     * -M_PI and M_PI
     *
     * @param lVariable (meters or radians)
     * @return true
     * @return false
     */
    bool isWithinConstraints(double lVariable) const;

    /**
     * @brief Constrains a variable to be between lMinValue and lMaxValue
     * In case of a REVOLUTE joint, the variable is first translated to fall
     * between -M_PI and M_PI
     * @param lVariable Variable to constrain, if variable already falls between
     * lMinValue and lMaxValue the variable is returned unchanged (meters or radians)
     * @return double
     */
    double constrainVariable(double lVariable) const;

      private:
    /**
     * @brief Calculates the transformation Matrix using the
     * "modified" denavit-hartenberg formula
     * @see
     * https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters
     * @param lD_m
     * @param lTheta_rad
     * @return Matrix<double, 4, 4>
     */
    Matrix<double, 4, 4> calculateTransformationMatrix(double lD_m,
                                                       double lTheta_rad) const;

    double mA_m;
    double mAlpha_rad;
    double mD_m;
    double mTheta_rad;
    eJoint  mType;
    double mMinValue; /// Unit of this variable depends on the value of mType
    double mMaxValue; /// Unit of this variable depends on the value of mType
  };

} // namespace kinematics
#endif // KINEMATICS_LINK_HPP
