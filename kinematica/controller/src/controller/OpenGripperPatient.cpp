#include "controller/OpenGripperPatient.hpp"
#include "controller/Init.hpp"

namespace controller
{
  void OpenGripperPatient::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<Init>());
  }
} // namespace controller