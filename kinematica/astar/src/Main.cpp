#include "../include/astar/AStar.hpp"
#include "../include/astar/Graph.hpp"
#include "../include/obstacle/Obstacle.hpp"

#include <memory>

int main()
{
  std::shared_ptr<astar::Graph> lGraph = std::make_shared<astar::Graph>();
  lGraph->addObstacle(
      obstacle::Obstacle{ 0.0f, 0.0f, -0.2f, 0.2f, 0.2f, 0.2f });
  lGraph->addObstacle(
      obstacle::Obstacle{ 0.2f, 0.0f, -1.1f, 1.0f, 1.2f, 1.2f });

  astar::AStar astar(lGraph);

  astar::Path lP =
      astar.search(astar::Vertex(0, 0, 0), astar::Vertex(66, 38, -200));
  for (const astar::Vertex& lV : lP)
  {
    std::cout << lV << std::endl;
  }
  return 0;
}
