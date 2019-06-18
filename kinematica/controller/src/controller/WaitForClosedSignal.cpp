#include "controller/WaitForClosedSignal.hpp"
#include "controller/CloseGripperPatient.hpp"
namespace controller
{
  void WaitForClosedSignal::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<CloseGripperPatient>());
  }
} // namespace controller