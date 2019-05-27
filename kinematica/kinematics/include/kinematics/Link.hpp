#ifndef KINEMATICS_LINK_HPP
#define KINEMATICS_LINK_HPP

#include "kinematics/Joint.hpp"
#include "matrix/Matrix.hpp"

namespace kinematics
{

  /**
   * @brief Representation of a Link in a robotarm using Denavit
   * Hartenberg parameters
   * @author Emiel Bosman
   * @see https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
   */
  class Link
  {
      public:
    Link();
    /**
     * @brief Construct a new Link object
     * @author Emiel Bosman
     *
     * @param aA_m Link length
     * @param aAlpha_rad Link twist
     * @param aD_m Link offset
     * @param aTheta_rad Joint angle
     * @param aType
     * @param aMinValue Radians if aType is REVOLUTE, meters if aType is
     * PRISMATIC, unused if aType is STATIC
     * @param aMaxValue Radians if aType is REVOLUTE, meters if aType is
     * PRISMATIC, unused if aType is STATIC
     */
    Link(double aA_m,
         double aAlpha_rad,
         double aD_m,
         double aTheta_rad,
         eJoint aType,
         double aMinValue = 0,
         double aMaxValue = 0);

    /**
     * @brief Construct a new Link object, sets the static value
     * to 0
     * @author Emiel Bosman
     * @param aA_m Link Length
     * @param aAlpha_rad Link Twist
     * @param aConstant Value of the constant value (mD_m or
     * mTheta_rad)
     * @param aType Type of Joint (eJoint::PRISMATIC sets
     * mTheta_rad, eJoint::REVOLUTE sets mD_rad)
     * @param aMinValue
     * @param aMaxValue
     */
    Link(double aA_m,
         double aAlpha_rad,
         double aConstant,
         eJoint aType,
         double aMinValue,
         double aMaxValue);
    ~Link() = default;
    Link(const Link&) = default;
    Link& operator=(const Link&) = default;

    /**
     * @brief Get the Transformation Matrix for a given Link
     * @author Emiel Bosman
     * @param aVariable Changing value for this Link (meters or radians)
     * @return Matrix<double, 4, 4> The TransformationMatrix
     * ~~~
     * [ r00 r11 r12 tx ]
     * [ r10 r21 r22 ty ]
     * [ r20 r31 r32 tz ]
     * [ 0   0   0   1  ]
     * ~~~
     */
    Matrix<double, 4, 4> transformationMatrix(double aVariable) const;

    /**
     * @brief Generate a random variable that falls within
     * mMinValue and mMaxValue of this link
     *
     * @return double
     */
    double generateRandomVariable() const;

    eJoint getType() const;

    /**
     * @brief Checks wether variable falls within aMinValue and aMaxValue
     * In case of a REVOLUTE joint, variable is first translated to fall between
     * -M_PI and M_PI
     * @author Emiel Bosman
     *
     * @param aVariable (meters or radians)
     * @return true
     * @return false
     */
    bool isWithinConstraints(double aVariable) const;

      private:
    /**
     * @brief Calculates the transformation Matrix using the
     * "modified" denavit-hartenberg formula
     * @author Emiel Bosman
     * @see
     * https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters
     * @param aD_m
     * @param aTheta_rad
     * @return Matrix<double, 4, 4>
     */
    Matrix<double, 4, 4> calculateTransformationMatrix(double aD_m,
                                                       double aTheta_rad) const;
    double mA_m;
    double mAlpha_rad;
    double mD_m;
    double mTheta_rad;
    eJoint mType;
    double mMinValue; /// Unit of this variable depends on the value of mType
    double mMaxValue; /// Unit of this variable depends on the value of mType
  };

} // namespace kinematics
#endif // KINEMATICS_LINK_HPP
