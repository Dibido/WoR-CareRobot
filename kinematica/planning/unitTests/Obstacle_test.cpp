#include "planning/AStar.hpp"
#include "planning/Graph.hpp"
#include "planning/Obstacle.hpp"
#include "planning/Vertex.hpp"
#include "gtest/gtest.h"
#include <iostream>

// LEGEND:
// P = Point from the found path
// S = Start point
// G = GOAL
// | = OBSTACLE
// ― = OBSTACLE
// . = FREE POINT

namespace planning
{
  TEST(ObstaclePathfinding, 2Dpath_1)
  {
    /*
      . . P S . . . | ― ― ― .
      . P . . . . . . . . . .
      P . . ― ― ― ― ― . . . .
      P . . . . . . . . . P G
      . P P P P P P P P P . .
    */
    std::shared_ptr<Graph> lGraph = std::make_shared<Graph>();
    lGraph->addObstacle(Obstacle{ 0.0f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.1f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.2f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.3f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.5f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.6f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.7f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });

    AStar astar(lGraph);
    Path lExpectedPath{ Vertex{ 0, 0, 0 },    Vertex{ -10, 0, 0 },
                        Vertex{ -20, 10, 0 }, Vertex{ -20, 20, 0 },
                        Vertex{ -20, 30, 0 }, Vertex{ -10, 40, 0 },
                        Vertex{ 0, 40, 0 },   Vertex{ 10, 40, 0 },
                        Vertex{ 20, 40, 0 },  Vertex{ 30, 40, 0 },
                        Vertex{ 40, 40, 0 },  Vertex{ 50, 40, 0 },
                        Vertex{ 60, 30, 0 },  Vertex{ 70, 30, 0 } };
    Path lP = astar.search(Vertex(0, 0, 0), Vertex(70, 30, 0));
    EXPECT_EQ(lExpectedPath, lP);
  }

  TEST(ObstaclePathfinding, 2DPath_2)
  {
    /*
        S . . . | ― ― ―
        . P . . | . . .
        . P . ― ― . . .
        . P . . . . . .
        . . P P P P P G
    */
    std::shared_ptr<Graph> lGraph = std::make_shared<Graph>();
    lGraph->addObstacle(Obstacle{ 0.3f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.1f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.5f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(Obstacle{ 0.6f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    AStar astar(lGraph);
    Path lExpectedPath{ Vertex{ 0, 0, 0 },   Vertex{ 10, 10, 0 },
                        Vertex{ 10, 20, 0 }, Vertex{ 10, 30, 0 },
                        Vertex{ 20, 40, 0 }, Vertex{ 30, 40, 0 },
                        Vertex{ 40, 40, 0 }, Vertex{ 50, 40, 0 },
                        Vertex{ 60, 40, 0 }, Vertex{ 70, 40, 0 } };
    Path lP = astar.search(Vertex(0, 0, 0), Vertex(70, 40, 0));
    EXPECT_EQ(lExpectedPath, lP);
  }

  TEST(ObstaclePathfinding, 2DPath_3)
  {
    /*
      ― ― ― ― ― ― ― ― ― ― ―
      | S | . . . | . . . |
      | . | . | . | . | . |
      | . | . | . | . | . |
      | . . . | . . . | . |
      ― ― ― ― ― ― ― ― ― ― ―
    */
    std::shared_ptr<Graph> lGraph = std::make_shared<Graph>();
    lGraph->addObstacle(Obstacle{ 0.0f, -0.1f, 0.0f, 10.0f, 0.1f, 10.0f });
    lGraph->addObstacle(Obstacle{ 0.9f, 0.4f, 0.0f, 10.0f, 0.1f, 10.0f });
    lGraph->addObstacle(Obstacle{ -0.1f, -0.1f, 0.0f, 0.1f, 10.0f, 10.0f });
    lGraph->addObstacle(Obstacle{ 0.9f, -0.1f, 0.0f, 0.1f, 10.0f, 10.0f });
    lGraph->addObstacle(Obstacle{ 0.1f, 0.1f, 0.0f, 0.1f, 0.4f, 10.0f });
    lGraph->addObstacle(Obstacle{ 0.3f, 0.2f, 0.0f, 0.1f, 0.4f, 10.0f });
    lGraph->addObstacle(Obstacle{ 0.5f, 0.1f, 0.0f, 0.1f, 0.4f, 10.0f });
    lGraph->addObstacle(Obstacle{ 0.7f, 0.2f, 0.0f, 0.1f, 0.4f, 10.0f });
    AStar astar(lGraph);
    Path lExpectedPath{};

    Path lP = astar.search(Vertex(0, 0, 0), Vertex(80, 30, 0));
    EXPECT_EQ(lExpectedPath, lP);
  }

  TEST(ObstaclePathfinding, 3DPath_1)
  {

    /*          Layer -1
        ― ― ― ― ― ― ― ― ― ― ― ―
        | ― ― ― ― ― ― ― ― ― ― |
        | ― ― ― ― ― ― ― ― ― ― |
        | ― ― ― ― ― ― ― ― ― ― |
        | ― ― ― ― ― ― ― ― ― ― |
        ― ― ― ― ― ― ― ― ― ― ― ―
                Layer 0
        ― ― ― ― ― ― ― ― ― ― ― ―
        | S P P P P P P P P . |
        | ― ― ― ― ― ― ― ― ― ― |
        | . . . . . . . . . . |
        | . . . . . . . . . . |
        ― ― ― ― ― ― ― ― ― ― ― ―
                Layer 1
        ― ― ― ― ― ― ― ― ― ― ― ―
        | . . . . . . . . . P |
        | ― ― ― ― ― ― ― ― ― ― |
        | . . . . . . ― ― . . |
        | G P P P P . ― ― . . |
        ― ― ― ― ― ― ― ― ― ― ― ―
                Layer 2
        ― ― ― ― ― ― ― ― ― ― ― ―
        | . . . . . . . ― ― . |
        | . . ― ― ― ― ― ― ― P |
        | ― ― ― ― ― P P P P . |
        | . . . . ― . . . . . |
        ― ― ― ― ― ― ― ― ― ― ― ―
                Layer 3
        ― ― ― ― ― ― ― ― ― ― ― ―
        | ― ― ― ― ― ― ― ― ― ― |
        | ― ― ― ― ― ― ― ― ― ― |
        | ― ― ― ― ― ― ― ― ― ― |
        | ― ― ― ― ― ― ― ― ― ― |
        ― ― ― ― ― ― ― ― ― ― ― ―
    */
    std::shared_ptr<Graph> lGraph = std::make_shared<Graph>();
    lGraph->addObstacle(Obstacle{ 0.5f, -0.1f, 0.0f, 1.2f, 0.1f, 0.6f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.1f, 0.0f, 2.0f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.1f, 0.1f, 2.0f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.5f, -0.1f, 0.0f, 1.2f, 0.1f, 0.6f });
    lGraph->addObstacle(Obstacle{ -0.1f, 0.1f, 0.0f, 0.1f, 0.8f, 0.6f });
    lGraph->addObstacle(Obstacle{ 1.0f, 0.1f, 0.0f, 0.1f, 0.8f, 0.6f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.1f, 0.3f, 2.0f, 1.0f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.1f, -0.1f, 2.0f, 1.0f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.6f, 0.2f, 0.1f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.7f, 0.2f, 0.1f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.6f, 0.3f, 0.1f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.7f, 0.3f, 0.1f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.7f, 0.1f, 0.2f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.8f, 0.1f, 0.2f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.3f, 0.1f, 0.2f, 0.8f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.2f, 0.2f, 0.2f, 0.5f, 0.1f, 0.1f });
    lGraph->addObstacle(Obstacle{ 0.4f, 0.3f, 0.2f, 0.1f, 0.1f, 0.1f });
    AStar astar(lGraph);
    Path lExpectedPath{
    };
    Path lP = astar.search(Vertex(0, 0, 0), Vertex(0, 30, 10));
    EXPECT_EQ(lExpectedPath, lP);
  }

  TEST(ObstaclePathfinding, NoPath)
  {
    /*
       | ― |
       | S | G
       | ― |
    */
    std::shared_ptr<Graph> lGraph = std::make_shared<Graph>();
    lGraph->addObstacle(Obstacle{ 0.0f, -0.1f, 0.0f, 10.0f, 0.1f, 10.0f });
    lGraph->addObstacle(Obstacle{ 0.0f, 0.1f, 0.0f, 10.0f, 0.1f, 10.0f });
    lGraph->addObstacle(Obstacle{ -0.1f, -0.1f, 0.0f, 10.0f, 0.3f, 10.0f });
    lGraph->addObstacle(Obstacle{ 0.1f, -0.1f, 0.0f, 10.0f, 0.3f, 10.0f });

    AStar astar(lGraph);

    Path lP = astar.search(Vertex(0, 0, 0), Vertex(80, 30, 0));
    EXPECT_EQ(( unsigned )0, lP.size());
  }

  TEST(ObstaclePathfinding, DifferentStartPoint)
  {
    /*
       G P S
    */
    std::shared_ptr<Graph> lGraph = std::make_shared<Graph>();
    AStar astar(lGraph);
    Path lExpectedPath{ Vertex{ 20, 0, 0 }, Vertex{ 10, 0, 0 },
                        Vertex{ 0, 0, 0 } };
    Path lP = astar.search(Vertex(20, 0, 0), Vertex(0, 0, 0));
    EXPECT_EQ(lExpectedPath, lP);
  }
} // namespace planning