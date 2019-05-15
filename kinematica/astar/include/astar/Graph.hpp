/**
 * @file Graph.hpp
 * @author Gianni Monteban (G.Monteban@student.han.nl)
 * @brief
 * @version 0.1
 * @date 2019-05-15
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef GRAPH_HPP
#define GRAPH_HPP
#include "../obstacle/Obstacle.hpp"
#include "Vertex.hpp"
#include <vector>

namespace astar
{
  const short STEP = 10;
  const short NR_OF_NEIGHBOURS = 9;
  /**
   * @brief
   *
   */
  class Graph
  {
      private:
    std::vector<obstacle::Obstacle> obstacles;
    Vertex endPoint;

      public:
    /**
     * @brief Construct a new Graph object
     *
     */
    Graph();

    /**
     * @brief Destroy the Graph object
     *
     */
    ~Graph();

    /**
     * @brief Set the End Point object
     *
     * @param aVertex
     */
    void setEndPoint(const Vertex& aVertex);

    /**
     * @brief
     *
     * @param obstacle
     */
    void addObstacle(const obstacle::Obstacle& obstacle);

    /**
     * @brief
     *
     * @param obstacle
     */
    void removeObstacle(const obstacle::Obstacle& obstacle);

    /**
     * @brief
     *
     */
    void removeAllObstacles();

    /**
     * @brief
     *
     * @param aVertex
     * @return std::vector<Vertex>
     */
    std::vector<Vertex> calculateNeigbours(const Vertex& aVertex);
  };

} // namespace astar

#endif // GRAPH_HPP