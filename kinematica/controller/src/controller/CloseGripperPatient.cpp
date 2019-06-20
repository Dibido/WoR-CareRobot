
#include "controller/CloseGripperPatient.hpp"
#include "controller/MoveToDropTable.hpp"
#include "controller/ControllerConsts.hpp"
namespace controller
{

    void CloseGripperPatient::entryAction(Context* aContext)
  {
    aContext->gripperData() = robotcontroller::GripperData(0,
        cSpeedFactor);
    aContext->robotGripper()->moveGripper(aContext->gripperData());
  }
  void CloseGripperPatient::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<MoveToDropTable>());
  }
} // namespace controller