#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class CollisionBehaviorModel : public ModelPlugin
  {
      public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    // Called by the world update start event
      public:
    void OnUpdate();

    // Pointer to the model
      private:
    physics::ModelPtr mModel;

    // Pointer to the update event connection
      private:
    event::ConnectionPtr mUpdateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CollisionBehaviorModel)
} // namespace gazebo