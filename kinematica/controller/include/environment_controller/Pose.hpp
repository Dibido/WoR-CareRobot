/**
 * @file Pose.hpp
 * @author Brandon Geldof
 * @brief the header file for the pose data struct
 * @version 0.1
 * @date 2019-06-03
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef POSE_HPP
#define POSE_HPP

#include <stdexcept>

#include "environment_controller/Position.hpp"
#include "environment_controller/Rotation.hpp"
#include "ros/ros.h"

namespace environment_controller
{
  /**
   * @brief The data struct for a pose. A pose consists of a position (x,y,z)
   * and a rotation (x,y,z,w)
   * @author Brandon Geldof
   */
  struct Pose
  {
      public:
    /**
     * @brief Construct a new Pose object
     *
     * @param aPosition See Position.hpp
     * @param aRotation See Rotation.hpp
     */
    Pose(const Position& aPosition, const Rotation& aRotation);
    /**
     * @brief Getter & setter
     *
     * @return Position&
     */
    Position& position();
    const Position& position() const;
    /**
     * @brief Getter & setter
     *
     * @return Rotation&
     */
    Rotation& rotation();
    const Rotation& rotation() const;

      private:
    Position mPosition;
    Rotation mRotation;
  };
} // namespace environment_controller

#endif // POSE_HPP