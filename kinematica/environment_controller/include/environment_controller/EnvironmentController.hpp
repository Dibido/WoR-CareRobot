/**
 * @file EnvironmentController.hpp
 * @author Martijn Vogelaar
 * @brief
 * @version 0.1
 * @date 2019-05-22
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef ENVIRONMENT_CONTROLLER_HPP
#define ENVIRONMENT_CONTROLLER_HPP

#include "IObstacles.hpp"
#include "controller/Context.hpp"
#include "environment_controller/Cup.hpp"
#include "ros/ros.h"

namespace environment_controller
{
  class EnvironmentController
  {

      public:
    EnvironmentController(const std::shared_ptr<controller::Context>& aContext);

    /**
     * @brief Destroy the EnvironmentController object
     *
     * @author Martijn Vogelaar
     */
    ~EnvironmentController() = default;

    /**
     * @brief provideObstacles will pass along all obstacles to the Controller
     *
     * @param aObstacles Obstacles which will be passed on.
     */
    void provideObstacles(const Obstacles& aObstacles);

    /**
     * @brief executeHardstop Will pass along the executeHardstop to the
     * controller
     *
     * @param hardstop If hardstop is true a hardstop has to be executed. If
     * hardstop is false the hardstop has to be lifted.
     */
    void executeHardstop(bool hardstop);

    /**
     * @brief provide the cup position so the robot can move
     *
     * @param aCup the cup to move to
     */
    void provideCup(const Cup& aCup);

      private:
    std::shared_ptr<controller::Context> mContext;
  };
} // namespace environment_controller

#endif // ENVIRONMENT_CONTROLLER_HPP