/**
 * @file Graph.hpp
 * @author Gianni Monteban (G.Monteban@student.han.nl)
 * @brief the header file for the graph class
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
  const uint8_t cStep = 10;
  const uint8_t cNrOfNeighbours = 9;
  const uint8_t cNrOfDimensionLayers = 3;
  /**
   * @brief The graph is build in Centimeters
   *
   * @author Gianni Monteban
   */
  class Graph
  {
      public:
    /**
     * @brief Construct a new Graph object
     * @author Gianni Monteban
     */
    Graph();

    /**
     * @brief Destroy the Graph object
     * @author Gianni Monteban
     */
    virtual ~Graph() = default;

    /**
     * @brief Set the End Point object
     *
     * @param aVertex the endPoint in the graph
     *
     * @author Gianni Monteban
     */
    void setEndPoint(const Vertex& aVertex);

    /**
     * @brief add an obstacle in the graph
     *
     * @param obstacle the obstacle to add
     *
     * @author Gianni Monteban
     */
    void addObstacle(const obstacle::Obstacle& obstacle);

    /**
     * @brief remove a specified obstacle from the graph
     *
     * @param obstacle the obstacle to remove from the graph
     *
     * @author Gianni Monteban
     */
    void removeObstacle(const obstacle::Obstacle& obstacle);

    /**
     * @brief remove all obstacles
     *
     * @author Gianni Monteban
     */
    void removeAllObstacles();

    /**
     * @brief calculate which vertexes are neighbours of the given vertex
     *
     * @param aVertex the vertex to calculate neighbours for
     * @return std::vector<Vertex> the list of neighbours
     *
     * @author Gianni Monteban
     */
    std::vector<Vertex> calculateNeighbours(const Vertex& aVertex);

      private:
    std::vector<obstacle::Obstacle> mObstacles;
    Vertex mEndPoint;
    /**
     * @brief Checks whether if the given point is within any of the known obstacles.
     * 
     * @param aVertex Point which will be checked for "collision"
     * @return true True will be returned if there is no obstacle which collides with the given point.
     * @return false False will be returned if there is an obstacle which collides with the given point.
     * 
     * @author Martijn Vogelaar
     */
    bool isPointInAnObstacle(const Vertex& aVertex) const;
  };

} // namespace astar

#endif // GRAPH_HPP