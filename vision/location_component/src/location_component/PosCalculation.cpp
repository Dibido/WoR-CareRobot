#include "location_component/PosCalculation.hpp"

namespace location_component
{

  PosCalculation::PosCalculation()
  {
  }

  PosCalculation::~PosCalculation()
  {
  }

  void PosCalculation::getWorldPos()
  {
	
	xpos * (100 / cAGVDepth) * cCupHeight;
	ypos * (100 / cAGVDepth) * cCupHeight;
  }


} // namespace location_component
