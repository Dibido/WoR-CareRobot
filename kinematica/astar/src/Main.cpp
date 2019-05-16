#include "../include/astar/AStar.hpp"
#include "../include/astar/Graph.hpp"

#include <memory>

int main()
{
  std::shared_ptr<astar::Graph> graph = std::make_shared<astar::Graph>();
  astar::AStar astar(graph);

  astar::Path p =
      astar.search(astar::Vertex(0, 0, 0), astar::Vertex(25, 16, -200));
  for (astar::Vertex v : p)
  {
    std::cout << v << std::endl;
  }
  return 0;
}
