#include "controller/OpenGripperTable.hpp"
#include "controller/Init.hpp"

namespace controller
{
  void OpenGripperTable::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<Init>());
  }
} // namespace controller