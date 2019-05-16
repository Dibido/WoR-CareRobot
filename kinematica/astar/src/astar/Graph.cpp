#include "../../include/astar/Graph.hpp"

#include <algorithm>
#include <unistd.h>
namespace astar
{

  Graph::Graph() : endPoint(0, 0, 0)
  {
  }

  void Graph::setEndPoint(const Vertex& aVertex)
  {
    endPoint = aVertex;
  }

  void Graph::addObstacle(const obstacle::Obstacle& obstacle)
  {
    obstacles.push_back(obstacle);
  }

  void Graph::removeObstacle(const obstacle::Obstacle& obstacle)
  {
    obstacles.erase(
        std::remove_if(obstacles.begin(), obstacles.end(),
                       [obstacle](const obstacle::Obstacle anObstacle) {
                         return obstacle == anObstacle;
                       }),
        obstacles.end());
  }

  void Graph::removeAllObstacles()
  {
    obstacles.clear();
  }

  std::vector<Vertex> Graph::calculateNeigbours(const Vertex& aVertex)
  {

    static int xOffset[] = { 0,         STEP,      STEP,      STEP, 0,
                             -1 * STEP, -1 * STEP, -1 * STEP, 0 };
    static int yOffset[] = { STEP,      STEP, 0,    -1 * STEP, -1 * STEP,
                             -1 * STEP, 0,    STEP, 0 };
    static int zOffset[] = { -1 * STEP, 0, STEP };

    std::vector<Vertex> neighbours;
    Vertex difference = Vertex(aVertex.x - endPoint.x, aVertex.y - endPoint.y,
                               aVertex.z - endPoint.z);

    if (((difference.x >= 0 && difference.x <= STEP) ||
         (difference.x <= 0 && difference.x >= (STEP * -1))) &&
        ((difference.y >= 0 && difference.y <= STEP) ||
         (difference.y <= 0 && difference.y >= (STEP * -1))) &&
        ((difference.z >= 0 && difference.z <= STEP) ||
         (difference.z <= 0 && difference.z >= (STEP * -1))))
    {
      neighbours.push_back(endPoint);
      return neighbours;
    }

    for (int j = 0; j < 3; ++j)
    {
      for (int i = 0; i < NR_OF_NEIGHBOURS; ++i)
      {
        Vertex vertex(aVertex.x + xOffset[i], aVertex.y + yOffset[i],
                      aVertex.z + zOffset[j]);
        neighbours.push_back(vertex);
      }
    }

    return neighbours;
  }
} // namespace astar