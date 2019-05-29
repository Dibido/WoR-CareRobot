/**
 * @file SafetyController.hpp
 * @author Martijn Vogelaar
 * @brief
 * @version 0.1
 * @date 2019-05-22
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef SAFETY_CONTROLLER_HPP
#define SAFETY_CONTROLLER_HPP

#include "EnvironmentController.hpp"
#include "IObstacles.hpp"
#include "ros/ros.h"
#include <memory>

namespace environment_controller
{
  /**
   * @brief Safetycontroller is the class which will determine whether the
   received obstacles are a "threat" to the robotarm or not.

   * @author Martijn Vogelaar
   */
  class SafetyController
  {

      public:
    /**
     * @brief Construct a new Safety Controller object
     *
     * @param aEnvironmentController An Environment controller has to be
     * supplied to be able to send hardstops and obstacles to.
     */
    SafetyController(
        const std::shared_ptr<EnvironmentController>& aEnvironmentController);

    /**
     * @brief Destroy the EnvironmentController object
     *
     * @author Martijn Vogelaar
     */
    ~SafetyController() = default;

    /**
     * @brief executeHardstopOnObstacleThret checks whether the received
     * obstacles are a threat or not.
     *
     * @param obstacles Obstacles which will be checked w
     */
    void executeHardstopOnObstacleThret(const Obstacles& aObstacles);

      private:
    std::shared_ptr<EnvironmentController> mEnvironmentController;

    /**
     * @brief isObstacleToClose check whether the given object is too close to
     * the robotarm.
     *
     * @param aObstacle Obstacle which will be checked
     * @return true True will be returned if the obstacle is too close to the
     * robotarm
     * @return false False will be returned if the obstacle is not too close to
     * the robotarm.
     */
    bool isObstacleToClose(const Obstacle& aObstacle);
  };
} // namespace environment_controller

#endif // SAFETY_CONTROLLER_HPP