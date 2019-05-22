#include "planning/Graph.hpp"
#include "planning/Vertex.hpp"
#include "gtest/gtest.h"

namespace planning
{
  TEST(GraphConstructor, DefaultConstructor)
  {
    EXPECT_NO_THROW(Graph());
  }

  TEST(GraphRemoveObstacle, RemoveObastacleDontExist)
  {
    environment_controller::Obstacle ob;
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
    environment_controller::Obstacle ob;
    Graph graph;

    EXPECT_NO_THROW(graph.addObstacle(ob));
  }

  TEST(GraphAddObstacle, addObstacleAndRemove)
  {
    environment_controller::Obstacle ob;
    Graph graph;

    graph.addObstacle(ob);

    EXPECT_NO_THROW(graph.removeObstacle(ob));
  }

  TEST(GraphSetEndPoint, testEndPoint)
  {
    Graph graph;

    Vertex v(-10, -20, -10);

    graph.setEndPoint(v);
    EXPECT_EQ(v, graph.calculateNeighbours(Vertex(-10, -20, -15)).at(0));
    EXPECT_NE(v, graph.calculateNeighbours(Vertex(0, 0, 0)).at(0));
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
    EXPECT_NE(controlList, graph.calculateNeighbours(Vertex(0, 10, 0)));
  }
} // namespace planning
