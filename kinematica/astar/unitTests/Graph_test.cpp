#include "astar/Graph.hpp"
#include "astar/Vertex.hpp"
#include "gtest/gtest.h"

namespace astar
{
  TEST(GraphConstructor, DefaultConstructor)
  {
    EXPECT_NO_THROW(Graph());
  }

  TEST(GraphRemoveObstacle, RemoveObastacleDontExist)
  {
    obstacle::Obstacle ob;
    Graph graph;

    EXPECT_NO_THROW(graph.removeObstacle(ob));
  }

  TEST(GraphRemoveObstacle, RemoveAllObstacles)
  {
    Graph graph;

    EXPECT_NO_THROW(graph.removeAllObstacles());
  }

  TEST(GraphAddObstacle, addObstacle)
  {
    obstacle::Obstacle ob;
    Graph graph;

    EXPECT_NO_THROW(graph.addObstacle(ob));
  }

  TEST(GraphAddObstacle, addObstacleAndRemove)
  {
    obstacle::Obstacle ob;
    Graph graph;

    graph.addObstacle(ob);

    EXPECT_NO_THROW(graph.removeObstacle(ob));
  }

  TEST(GrapSetEndPoint, testEndPoint)
  {
    Graph graph;

    Vertex v(-10, -20, -10);

    graph.setEndPoint(v);
    v.z -= 5;
    EXPECT_EQ(v, graph.calculateNeighbours(v).at(0));
  }

  TEST(GraphCaclulateNeighbours, calculateCorrectNeighbours)
  {
    std::vector<Vertex> controlList = {
      Vertex(0, 10, -10),   Vertex(10, 10, -10),  Vertex(10, 0, -10),
      Vertex(10, -10, -10), Vertex(0, -10, -10),  Vertex(-10, -10, -10),
      Vertex(-10, 0, -10),  Vertex(-10, 10, -10), Vertex(0, 0, -10),
      Vertex(0, 10, 0),     Vertex(10, 10, 0),    Vertex(10, 0, 0),
      Vertex(10, -10, 0),   Vertex(0, -10, 0),    Vertex(-10, -10, 0),
      Vertex(-10, 0, 0),    Vertex(-10, 10, 0),   Vertex(0, 0, 0),
      Vertex(0, 10, 10),    Vertex(10, 10, 10),   Vertex(10, 0, 10),
      Vertex(10, -10, 10),  Vertex(0, -10, 10),   Vertex(-10, -10, 10),
      Vertex(-10, 0, 10),   Vertex(-10, 10, 10),  Vertex(0, 0, 10)
    };

    Graph graph;
    graph.setEndPoint(Vertex(20, 20, 20));

    EXPECT_EQ(controlList, graph.calculateNeighbours(Vertex(0, 0, 0)));
  }
} // namespace astar
