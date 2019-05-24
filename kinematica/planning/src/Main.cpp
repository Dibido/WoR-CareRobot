#include "planning/AStar.hpp"
#include "planning/Graph.hpp"
#include "planning/Obstacle.hpp"

#include <memory>

int main()
{
  std::shared_ptr<planning::Graph> lGraph = std::make_shared<planning::Graph>();
  lGraph->addObstacle(
      planning::Obstacle{ 0.0f, 0.0f, -0.2f, 0.2f, 0.2f, 0.2f });
  lGraph->addObstacle(
      planning::Obstacle{ 0.2f, 0.0f, -1.1f, 1.0f, 1.2f, 1.2f });

  planning::AStar astar(lGraph);

  planning::Path lP =
      astar.search(planning::Vertex(0, 0, 0), planning::Vertex(66, 38, -200));
  for (const planning::Vertex& lV : lP)
  {
    std::cout  << lV << std::endl;
  }
  return 0;
}
