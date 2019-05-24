#ifndef CONTEXT_HPP
#define CONTEXT_HPP

#include "State.hpp"
#include "kinematics/ConfigurationProvider.hpp"
#include "planning/AStar.hpp"
#include "planning/Graph.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
#include "robotcontroller/RobotGripperPublisher.hpp"
#include "robotcontroller/RobotStopPublisher.hpp"
#include <memory>
#include <vector>

namespace controller
{
  class State;
  /**
   * @class Context
   *
   * @brief Context is a class which gives the states a interface to the
   * "outside world".
   *
   */
  class Context
  {
      public:
    /**
     * @brief Construct a new Context object
     *
     */
    Context();

    /**
     * @brief Set the currentState by supplying a shared_ptr to a state.
     *
     * @param state
     */
    void setState(const std::shared_ptr<State>& state);
    /**
     * @brief Run is mthe function which takes care of the handling of the
     * EventQueue and calling the doActivity functions of the different states.
     */
    void run();

    /**
     * @brief getters & setters
     *
     */

    std::shared_ptr<planning::Graph>& graph();
    std::shared_ptr<planning::AStar>& astar();
    std::shared_ptr<robotcontroller::RobotControlPublisher>& robotControl();
    std::shared_ptr<robotcontroller::RobotGripperPublisher>& robotGripper();
    std::shared_ptr<robotcontroller::RobotStopPublisher>& robotStop();
    std::shared_ptr<kinematics::ConfigurationProvider>& configuration();

      private:
    std::shared_ptr<planning::Graph> mGraph;
    std::shared_ptr<planning::AStar> mAstar;
    ros::NodeHandle mRobotControlHandle;
    ros::NodeHandle mRobotGripperHandle;
    ros::NodeHandle mRobotStopHandle;
    std::shared_ptr<robotcontroller::RobotControlPublisher>
        mRobotControlPublisher;
    std::shared_ptr<robotcontroller::RobotGripperPublisher>
        mRobotGripperPublisher;
    std::shared_ptr<robotcontroller::RobotStopPublisher> mRobotStopPublisher;
    std::shared_ptr<kinematics::ConfigurationProvider> mConfigurationProvider;
    std::shared_ptr<State> mCurrentState;
  };
} // namespace controller
#endif // Context_HPP