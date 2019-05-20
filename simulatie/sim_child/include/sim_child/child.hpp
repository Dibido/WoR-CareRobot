#include <functional>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class Child : public ModelPlugin
  {
  public: 
    
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
    void onUpdate();

    virtual ~Child() = default;

  private: 
    // Pointer to the model
    physics::ModelPtr mModel;

    // Pointer to the update event connection
    event::ConnectionPtr mUpdateConnection;

    double mVelocity = 0;
    double mAngle = 0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Child)
}