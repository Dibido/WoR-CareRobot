#include "planning/AStar.hpp"
#include "planning/Graph.hpp"

#include "gtest/gtest.h"

namespace planning
{
  TEST(ASTAR, constructor)
  {
    std::shared_ptr<Graph> graph = std::make_shared<Graph>();
    EXPECT_NO_THROW(AStar astar(graph));
  }

  TEST(ASTAR, search)
  {
    Vertex v1(0, 0, 0);
    Vertex v2(100, 100, 100);

    std::shared_ptr<Graph> graph = std::make_shared<Graph>();
    AStar astar(graph);

    Path pG = { Vertex(0, 0, 0),    Vertex(10, 10, 10),   Vertex(20, 20, 20),
                Vertex(30, 30, 30), Vertex(40, 40, 40),   Vertex(50, 50, 50),
                Vertex(60, 60, 60), Vertex(70, 70, 70),   Vertex(80, 80, 80),
                Vertex(90, 90, 90), Vertex(100, 100, 100) };

    Path p = astar.search(v1, v2);

    EXPECT_EQ(p, pG);
    pG.push_back(Vertex(110, 110, 110));
    EXPECT_NE(p, pG);
  }
} // namespace astar
