

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "Edge.hpp"
#include "Graph.hpp"
#include "Vertex.hpp"

namespace astar
{
  typedef std::vector<Vertex> Path;
  typedef std::vector<Vertex> OpenSet;
  typedef std::set<Vertex, VertexLessIdCompare> ClosedSet;
  typedef std::map<Vertex, Vertex, VertexLessIdCompare> VertexMap;
  /**
   *
   */
  class AStar
  {
      public:
    /**
     * @brief Construct a new AStar object
     *
     * @param aGraph
     */
    AStar(const std::shared_ptr<Graph>& aGraph);

    /**
     * @brief Destroy the AStar object
     *
     */
    virtual ~AStar() = default;

    /**
     *
     */
    Path search(Vertex aStart, const Vertex& aGoal);
    /**
     *
     */
    void addToOpenSet(const Vertex& aVertex);
    /**
     *
     */
    void removeFromOpenSet(const Vertex& aVertex);
    /**
     *
     */
    void removeFromOpenSet(OpenSet::iterator& i);
    /**
     *
     */
    OpenSet::iterator findInOpenSet(const Vertex& aVertex);
    /**
     *
     */
    void removeFirstFromOpenSet();
    /**
     *
     */
    void addToClosedSet(const Vertex& aVertex);
    /**
     *
     */
    void removeFromClosedSet(const Vertex& aVertex);
    /**
     *
     */
    void removeFromClosedSet(ClosedSet::iterator& i);
    /**
     *
     */
    ClosedSet::iterator findInClosedSet(const Vertex& aVertex);

      protected:
    /**
     *
     */
    ClosedSet& getCS();
    /**
     *
     */
    const ClosedSet& getCS() const;
    /**
     *
     */
    OpenSet& getOS();
    /**
     *
     */
    const OpenSet& getOS() const;
    /**
     *
     */
    VertexMap& getPM();
    /**
     *
     */
    const VertexMap& getPM() const;

      private:
    /**
     * @brief Get the Neighbour Connections object
     *
     * @param aVertex
     * @return std::vector<Edge>
     */
    std::vector<Edge> getNeighbourConnections(const Vertex& aVertex);
    /**
     *
     */
    std::shared_ptr<Graph> graph;
    /**
     *
     */
    ClosedSet closedSet;
    /**
     *
     */
    OpenSet openSet;
    /**
     *
     */
    VertexMap predecessorMap;

    mutable std::recursive_mutex openSetMutex;
    mutable std::recursive_mutex closedSetMutex;
    mutable std::recursive_mutex predecessorMapMutex;

  }; // class AStar
} // namespace astar

#endif // ASTAR_HPP