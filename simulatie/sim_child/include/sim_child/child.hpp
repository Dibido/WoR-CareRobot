/**
 * @file child.hpp
 * @author Maurits Muijsert (MPW.Muijsert@student.han.nl)
 * @brief gazebo plugin for simulating a moving child
 * @version 0.1
 * @date 2019-05-20
 *
 * @copyright Copyright (c) 2019
 *
 */
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>

namespace gazebo
{
  /**
   * @brief gazebo plugin for simulating a running child.
   *
   */
  class Child : public ModelPlugin
  {
      public:
    /**
     * @brief Called once by when loading in the plugin. loads the config
     * parameters of
     *
     * @param _parent
     * @param _sdf
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /**
     * @brief Called by the world update start event.
     *
     */
    void onUpdate();

    /**
     * @brief Destroy the Child object
     *
     */
    virtual ~Child() = default;

      private:
    physics::ModelPtr mModel;
    event::ConnectionPtr mUpdateConnection;
    double mVelocity = 0;
    double mAngle = 0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Child)
} // namespace gazebo