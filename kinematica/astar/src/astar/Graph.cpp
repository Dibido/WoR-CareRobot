#include "../../include/astar/Graph.hpp"

#include <algorithm>
#include <unistd.h>
namespace astar
{

  Graph::Graph() : mEndPoint(0, 0, 0)
  {
  }

  void Graph::setEndPoint(const Vertex& aVertex)
  {
    mEndPoint = aVertex;
  }

  void Graph::addObstacle(const obstacle::Obstacle& obstacle)
  {
    mObstacles.push_back(obstacle);
  }

  void Graph::removeObstacle(const obstacle::Obstacle& obstacle)
  {
    mObstacles.erase(
        std::remove_if(mObstacles.begin(), mObstacles.end(),
                       [obstacle](const obstacle::Obstacle anObstacle) {
                         return obstacle == anObstacle;
                       }),
        mObstacles.end());
  }

  void Graph::removeAllObstacles()
  {
    mObstacles.clear();
  }

  std::vector<Vertex> Graph::calculateNeighbours(const Vertex& aVertex)
  {
    const static short lXOffset[] = { 0,          cStep,      cStep,
                                      cStep,      0,          -1 * cStep,
                                      -1 * cStep, -1 * cStep, 0 };
    const static short lYOffset[] = { cStep,      cStep,      0,
                                      -1 * cStep, -1 * cStep, -1 * cStep,
                                      0,          cStep,      0 };
    const static short lZOffset[] = { -1 * cStep, 0, cStep };

    std::vector<Vertex> lNeighbours;
    Vertex lDifference =
        Vertex(aVertex.x - mEndPoint.x, aVertex.y - mEndPoint.y,
               aVertex.z - mEndPoint.z);

    if (((lDifference.x >= 0 && lDifference.x <= cStep) ||
         (lDifference.x <= 0 && lDifference.x >= (cStep * -1))) &&
        ((lDifference.y >= 0 && lDifference.y <= cStep) ||
         (lDifference.y <= 0 && lDifference.y >= (cStep * -1))) &&
        ((lDifference.z >= 0 && lDifference.z <= cStep) ||
         (lDifference.z <= 0 && lDifference.z >= (cStep * -1))))
    {
      lNeighbours.push_back(mEndPoint);
      return lNeighbours;
    }

    for (uint8_t j = 0; j < cNrOfDimensionLayers; ++j)
    {
      for (uint8_t i = 0; i < cNrOfNeighbours; ++i)
      {
        Vertex lVertex(aVertex.x + lXOffset[i], aVertex.y + lYOffset[i],
                       aVertex.z + lZOffset[j]);
        lNeighbours.push_back(lVertex);
      }
    }

    return lNeighbours;
  }
} // namespace astar