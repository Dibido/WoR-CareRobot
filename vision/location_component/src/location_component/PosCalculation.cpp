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
	
	xpos * (100 / AGVDepth) * CupHeight;
	ypos * (100 / AGVDepth) * CupHeight;
  }


} // namespace location_component
