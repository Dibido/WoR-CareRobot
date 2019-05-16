#include "../include/astar/AStar.hpp"
#include "../include/astar/Graph.hpp"

#include <memory>

int main()
{
  std::shared_ptr<astar::Graph> lGraph = std::make_shared<astar::Graph>();
  astar::AStar astar(lGraph);

  astar::Path lP =
      astar.search(astar::Vertex(0, 0, 0), astar::Vertex(66, 38, -200));
  for (const astar::Vertex& lV : lP)
  {
    std::cout << lV << std::endl;
  }
  return 0;
}
