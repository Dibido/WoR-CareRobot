#include "astar/AStar.hpp"
#include "astar/Graph.hpp"
#include "astar/Vertex.hpp"
#include "obstacle/Obstacle.hpp"
#include "gtest/gtest.h"
#include <iostream>

// LEGEND:
// P = Point from the found path
// S = Start point
// G = GOAL
// | = OBSTACLE
// ― = OBSTACLE
// . = FREE POINT

namespace astar
{
  TEST(ObstaclePathfinding, 2Dpath_1)
  {
    /*
        S . . . | ― ― ―
        . P P P P . . .
        ― ― ― ― ― P . .
        . . . . . . P G
    */
    std::shared_ptr<Graph> lGraph = std::make_shared<Graph>();
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.0f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.0f, 0.3f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.1f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.2f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.3f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.5f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.6f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    AStar astar(lGraph);
    Path lExpectedPath{ Vertex{ 0, 0, 0 },   Vertex{ 10, 0, 0 },
                        Vertex{ 20, 10, 0 }, Vertex{ 30, 10, 0 },
                        Vertex{ 40, 10, 0 }, Vertex{ 50, 20, 0 },
                        Vertex{ 60, 30, 0 }, Vertex{ 70, 30, 0 } };
    Path lP = astar.search(astar::Vertex(0, 0, 0), astar::Vertex(70, 30, 0));
    EXPECT_EQ(lExpectedPath, lP);

  }

  TEST(ObstaclePathfinding, 2DPath_2)
  {
    /*
        S . . . | ― ― ―
        . P . . | . . .
        ― ― P ― ― . . .
        . . . P P P P G
    */
    std::shared_ptr<Graph> lGraph = std::make_shared<Graph>();
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.0f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.0f, 0.3f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.1f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.3f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.2f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.1f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.5f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.6f, 0.0f, 0.0f, 0.1f, 0.1f, 1.0f });
    AStar astar(lGraph);
    Path lExpectedPath{ Vertex{ 0, 0, 0 },   Vertex{ 10, 10, 0 },
                        Vertex{ 20, 20, 0 }, Vertex{ 30, 30, 0 },
                        Vertex{ 40, 30, 0 }, Vertex{ 50, 30, 0 },
                        Vertex{ 60, 30, 0 }, Vertex{ 70, 30, 0 } };
    Path lP = astar.search(astar::Vertex(0, 0, 0), astar::Vertex(70, 30, 0));
    EXPECT_EQ(lExpectedPath, lP);
  }

  TEST(ObstaclePathfinding, 2DPath_3)
  {
    /*
      ― ― ― ― ― ― ― ― ― ― ―
      | S | P P P | P P P |
      | P | P | P | P | P |
      | P | P | P | P | P |
      | P P P | P P P | G |
      ― ― ― ― ― ― ― ― ― ― ―
    */
    std::shared_ptr<Graph> lGraph = std::make_shared<Graph>();
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.0f, -0.1f, 0.0f, 10.0f, 0.1f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.9f, 0.4f, 0.0f, 10.0f, 0.1f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ -0.1f, -0.1f, 0.0f, 0.1f, 10.0f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.9f, -0.1f, 0.0f, 0.1f, 10.0f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.1f, 0.1f, 0.0f, 0.1f, 0.4f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.3f, 0.2f, 0.0f, 0.1f, 0.4f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.5f, 0.1f, 0.0f, 0.1f, 0.4f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.7f, 0.2f, 0.0f, 0.1f, 0.4f, 10.0f });
    AStar astar(lGraph);
    Path lExpectedPath{ Vertex{ 0, 0, 0 },   Vertex{ 0, 10, 0 },
                        Vertex{ 0, 20, 0 },  Vertex{ 10, 30, 0 },
                        Vertex{ 20, 20, 0 }, Vertex{ 20, 10, 0 },
                        Vertex{ 30, 0, 0 },  Vertex{ 40, 10, 0 },
                        Vertex{ 40, 20, 0 }, Vertex{ 50, 30, 0 },
                        Vertex{ 60, 20, 0 }, Vertex{ 60, 10, 0 },
                        Vertex{ 70, 0, 0 },  Vertex{ 80, 10, 0 },
                        Vertex{ 80, 20, 0 }, Vertex{ 80, 30, 0 } };

    Path lP = astar.search(astar::Vertex(0, 0, 0), astar::Vertex(80, 30, 0));
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
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.5f, -0.1f, 0.0f, 1.2f, 0.1f, 0.6f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.1f, 0.0f, 2.0f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.1f, 0.1f, 2.0f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.5f, -0.1f, 0.0f, 1.2f, 0.1f, 0.6f });
    lGraph->addObstacle(
        obstacle::Obstacle{ -0.1f, 0.1f, 0.0f, 0.1f, 0.8f, 0.6f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 1.0f, 0.1f, 0.0f, 0.1f, 0.8f, 0.6f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.1f, 0.3f, 2.0f, 1.0f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.1f, -0.1f, 2.0f, 1.0f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.6f, 0.2f, 0.1f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.7f, 0.2f, 0.1f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.6f, 0.3f, 0.1f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.7f, 0.3f, 0.1f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.7f, 0.1f, 0.2f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.8f, 0.1f, 0.2f, 0.1f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.3f, 0.1f, 0.2f, 0.8f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.2f, 0.2f, 0.2f, 0.5f, 0.1f, 0.1f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.4f, 0.3f, 0.2f, 0.1f, 0.1f, 0.1f });
    AStar astar(lGraph);
    Path lExpectedPath{
      Vertex{ 0, 0, 0 },    Vertex{ 10, 0, 0 },   Vertex{ 20, 0, 0 },
      Vertex{ 30, 0, 0 },   Vertex{ 40, 0, 0 },   Vertex{ 50, 0, 0 },
      Vertex{ 60, 0, 0 },   Vertex{ 70, 0, 0 },   Vertex{ 80, 0, 10 },
      Vertex{ 90, 10, 20 }, Vertex{ 80, 20, 20 }, Vertex{ 70, 20, 20 },
      Vertex{ 60, 20, 20 }, Vertex{ 50, 20, 20 }, Vertex{ 40, 30, 10 },
      Vertex{ 30, 30, 10 }, Vertex{ 20, 30, 10 }, Vertex{ 10, 30, 10 },
      Vertex{ 0, 30, 10 }
    };
    Path lP = astar.search(astar::Vertex(0, 0, 0), astar::Vertex(0, 30, 10));
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
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.0f, -0.1f, 0.0f, 10.0f, 0.1f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.0f, 0.1f, 0.0f, 10.0f, 0.1f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ -0.1f, -0.1f, 0.0f, 10.0f, 0.3f, 10.0f });
    lGraph->addObstacle(
        obstacle::Obstacle{ 0.1f, -0.1f, 0.0f, 10.0f, 0.3f, 10.0f });

    AStar astar(lGraph);

    Path lP = astar.search(astar::Vertex(0, 0, 0), astar::Vertex(80, 30, 0));
    EXPECT_EQ(0, lP.size());
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
    Path lP = astar.search(astar::Vertex(20, 0, 0), astar::Vertex(0, 0, 0));
    EXPECT_EQ(lExpectedPath, lP);
  }
} // namespace astar