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

#include <map>

#include "IObstacles.hpp"
#include "controller/Context.hpp"
#include "environment_controller/Cup.hpp"
#include "environment_controller/Sensor.hpp"
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

    /**
     * @brief provide the goal position where a grabbed cup needs to be placed
     *
     * @param aPosition
     */
    void provideGoal(const Position& aPosition);

    /**
     * @brief provide the time after which the cup will be released
     *
     * @param aReleaseTime_s in seconds
     */
    void provideReleaseTime(const uint8_t aReleaseTime_s);

    void registerSensor(const Sensor& aSensor);

    const Sensor& getSensor(const uint8_t aSensorID) const;

      private:
    std::shared_ptr<controller::Context> mContext;
    std::map<uint8_t, Pose> mSensors;
  };
} // namespace environment_controller

#endif // ENVIRONMENT_CONTROLLER_HPP