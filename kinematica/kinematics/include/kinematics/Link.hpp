#ifndef KINEMATICS_LINK_HPP
#define KINEMATICS_LINK_HPP

#include "kinematics/Joint.hpp"
#include "matrix/Matrix.hpp"

namespace kinematics
{

  /**
   * @brief Representation of a Link in a robotarm using Denavit
   * Hartenberg parameters
   *
   */
  class Link
  {
      public:
    /**
     * @brief Construct a new Link object
     *
     * @param a Link length
     * @param alpha Link twist
     * @param d Link offset
     * @param theta Joint angle
     * @param type
     * @param minValue
     * @param maxValue
     */
    Link(double a,
         double alpha,
         double d,
         double theta,
         Joint type,
         double minValue = 0,
         double maxValue = 0);

    /**
     * @brief Construct a new Link object, sets the static value
     * to 0
     * @param a Link Length
     * @param alpha Link Twist
     * @param constant Value of the constant value (d or
     * theta)
     * @param type Type of Joint (Joint::PRISMATIC sets
     * offsetTheta, Joint::REVOLUTE sets offsetD)
     * @param minValue
     * @param maxValue
     */
    Link(double a,
         double alpha,
         double constant,
         Joint type,
         double minValue,
         double maxValue);
    ~Link() = default;
    Link(const Link&) = default;
    Link& operator=(const Link&) = default;

    /**
     * @brief Get the Transformation Matrix for a given Link
     * @param variable Changing value for this Link
     * @return Matrix<double, 4, 4> The TransformationMatrix
     * ~~~
     * [ r00 r11 r12 tx ]
     * [ r10 r21 r22 ty ]
     * [ r20 r31 r32 tz ]
     * [ 0   0   0   1  ]
     * ~~~
     */
    Matrix<double, 4, 4> transformationMatrix(double variable) const;

    double getA() const;
    double getAlpha() const;
    double getTheta() const;
    double getD() const;
    Joint getType() const;

    /**
     * @brief Checks wether variable falls within minValue and maxValue
     * In case of a REVOLUTE joint, variable is first translated to fall between
     * -M_PI and M_PI
     *
     * @param variable
     * @return true
     * @return false
     */
    bool isWithinConstraints(double variable) const;

    /**
     * @brief Constrains a variable to be between minValue and maxValue
     * In case of a REVOLUTE joint, the variable is first translated to fall
     * between -M_PI and M_PI
     * @param variable Variable to constrain, if variable already falls between
     * minValue and maxValue the variable is returned unchanged
     * @return double
     */
    double constrainVariable(double variable) const;

      private:
    /**
     * @brief Calculates the transformation Matrix using the
     * "modified" denavit-hartenberg formula
     * @see
     * https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters
     * @param d
     * @param theta
     * @return Matrix<double, 4, 4>
     */
    Matrix<double, 4, 4> calculateTransformationMatrix(double d,
                                                       double theta) const;

    const double a;
    const double alpha;
    double d;
    double theta;
    const Joint type;
    const double minValue;
    const double maxValue;
  };

} // namespace kinematics
#endif // KINEMATICS_LINK_HPP
