#include "controller/MoveToDropLocation.hpp"
#include "controller/OpenGripperTable.hpp"
namespace controller
{

  void MoveToDropTable::entryAction(Context* aContext)
  {
    kinematics::EndEffector lTargetLocation = kinematics::EndEffector(
        aContext->dropPosition().x_m(), aContext->dropPosition().y_m(),
        aContext->cup().object().height_m() + aContext->dropPosition().z_m(), 0,
        M_PI_2, M_PI_2);
    // TODO need to get new position to drop the cup
    mTrajectoryProvider.createTrajectory(aContext, lTargetLocation, mTrajectory,
                                         false, true);
    mArrivalTime = ros::Time::now();
  }
  void MoveToDropTable::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<OpenGripperTable>());
  }
} // namespace controller