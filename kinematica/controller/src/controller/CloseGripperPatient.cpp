
#include "controller/CloseGripperPatient.hpp"
#include "controller/MoveToDropTable.hpp"

namespace controller
{
  void CloseGripperPatient::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<MoveToDropTable>());
  }
} // namespace controller