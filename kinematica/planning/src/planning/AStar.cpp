#include "planning/AStar.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace planning
{

  double ActualCost(const Vertex& aStart, const Vertex& aGoal)
  {
    return std::sqrt((aStart.x - aGoal.x) * (aStart.x - aGoal.x) +
                     (aStart.y - aGoal.y) * (aStart.y - aGoal.y) +
                     (aStart.z - aGoal.z) * (aStart.z - aGoal.z));
  }

  double HeuristicCost(const Vertex& aStart, const Vertex& aGoal)
  {
    return std::sqrt((aStart.x - aGoal.x) * (aStart.x - aGoal.x) +
                     (aStart.y - aGoal.y) * (aStart.y - aGoal.y) +
                     (aStart.z - aGoal.z) * (aStart.z - aGoal.z));
  }

  Path ConstructPath(VertexMap& aPredecessorMap, const Vertex& aCurrentNode)
  {
    VertexMap::iterator i = aPredecessorMap.find(aCurrentNode);
    if (i != aPredecessorMap.end())
    {
      Path path = ConstructPath(aPredecessorMap, (*i).second);
      path.push_back(aCurrentNode);
      return path;
    }
    else
    {
      Path path;
      path.push_back(aCurrentNode);
      return path;
    }
  }

  AStar::AStar(const std::shared_ptr<Graph>& aGraph) : graph(aGraph)
  {
  }

  std::vector<Edge> AStar::getNeighbourConnections(const Vertex& aVertex)
  {
    std::vector<Edge> connections;
    const std::vector<Vertex>& neighbours = graph->calculateNeighbours(aVertex);
    for (const Vertex& vertex : neighbours)
    {
      connections.push_back(Edge(aVertex, vertex));
    }

    return connections;
  }

  Path AStar::search(Vertex aStart, const Vertex& aGoal)
  {
    getOS().clear();
    getCS().clear();
    getPM().clear();

    graph->setEndPoint(aGoal);

    aStart.actualCost = 0.0; // Cost from aStart along the best known path.
    aStart.heuristicCost =
        aStart.actualCost + HeuristicCost(aStart,
                                          aGoal); // Estimated total cost from
                                                  // aStart to aGoal through y.

    addToOpenSet(aStart);

    while (!openSet.empty())
    {
      Vertex current = *openSet.begin();

      if (current.equalPoint(aGoal))
      {
        return ConstructPath(predecessorMap, current);
      }
      else
      {
        addToClosedSet(current);
        removeFirstFromOpenSet();

        const std::vector<Edge>& connections = getNeighbourConnections(current);
        for (const Edge& connection : connections)
        {
          Vertex neighbour = connection.otherSide(current);

          // The new costs
          double calculatedActualNeighbourCost =
              current.actualCost + ActualCost(current, neighbour);
          double totalHeuristicCostNeighbour =
              calculatedActualNeighbourCost + HeuristicCost(neighbour, aGoal);

          OpenSet::iterator openVertex = findInOpenSet(neighbour);
          if (openVertex != openSet.end())
          {
            // if neighbour is
            // in the openSet we
            // may have found a
            // shorter via-route
            if ((*openVertex).heuristicCost <= totalHeuristicCostNeighbour)
            {
              continue;
            }
            else
            {
              removeFromOpenSet(openVertex);
            }
          }
          ClosedSet::iterator closedVertex = findInClosedSet(neighbour);
          if (closedVertex != closedSet.end())
          {
            // if neighbour is
            // in the closedSet
            // we may have found
            // a shorter
            // via-route
            if ((*closedVertex).heuristicCost <= totalHeuristicCostNeighbour)
            {
              continue;
            }
            else
            {
              removeFromClosedSet(closedVertex);
            }
          }

          neighbour.actualCost = calculatedActualNeighbourCost;
          neighbour.heuristicCost = totalHeuristicCostNeighbour;

          std::pair<VertexMap::iterator, bool> insertResult1 =
              predecessorMap.insert(std::make_pair(neighbour, current));
          if (insertResult1.second != true)
          {
            if (!(*insertResult1.first).first.equalPoint(neighbour))
            {
              std::ostringstream os;
              os << "*** (*insertResult1.first).first != neighbour:\n\t\t"
                 << (*insertResult1.first).first << " <- "
                 << (*insertResult1.first).second << "\n\t\t" << neighbour
                 << " <- " << current;
              throw std::runtime_error(os.str());
            }

            if ((*insertResult1.first).first.heuristicCost >
                neighbour.heuristicCost)
            {
              predecessorMap.erase(insertResult1.first);
              std::pair<VertexMap::iterator, bool> insertResult2 =
                  predecessorMap.insert(std::make_pair(neighbour, current));
              if (insertResult2.second != true)
              {
                std::ostringstream os;
                os << "**** Failed updating neighbour in the predecessorMap:  "
                   << neighbour << " <- " << current << "\n";
                if (insertResult2.first != predecessorMap.end())
                {
                  os << "\tcurrent value in the predecessorMap:  "
                     << (*insertResult2.first).first << " <- "
                     << (*insertResult2.first).second;
                }
                else
                {
                  os << "\tinvalid iterator ";
                }
                throw std::runtime_error(os.str());
              }
              else
              {
              }
            }
          }
          else
          {
          }

          // if neighbour is not in
          // openSet, add it to the
          // openSet which should always
          // be the case?
          openVertex = findInOpenSet(neighbour);
          if (openVertex == openSet.end())
          {
            addToOpenSet(neighbour);
          }
          else
          {
            std::cerr << "**** "
                         "Failed "
                         "adding "
                         "neighbour "
                         "to the "
                         "openSet:  "
                      << neighbour << std::endl;
          }
        }
        std::iter_swap(openSet.begin(),
                       std::min_element(openSet.begin(), openSet.end(),
                                        VertexLessCostCompare()));
      }
    }

    std::cerr << "**** No route from " << aStart << " to " << aGoal
              << std::endl;
    return Path();
  }

  void AStar::addToOpenSet(const Vertex& aVertex)
  {
    std::unique_lock<std::recursive_mutex> lock(openSetMutex);
    openSet.push_back(aVertex);
  }

  void AStar::removeFromOpenSet(const Vertex& aVertex)
  {
    std::unique_lock<std::recursive_mutex> lock(openSetMutex);
    OpenSet::iterator i = findInOpenSet(aVertex);
    removeFromOpenSet(i);
  }

  void AStar::removeFromOpenSet(OpenSet::iterator& i)
  {
    std::unique_lock<std::recursive_mutex> lock(openSetMutex);
    openSet.erase(i);
  }

  OpenSet::iterator AStar::findInOpenSet(const Vertex& aVertex)
  {
    std::unique_lock<std::recursive_mutex> lock(openSetMutex);
    return std::find_if(
        openSet.begin(), openSet.end(),
        [aVertex](const Vertex& rhs) { return aVertex.equalPoint(rhs); });
  }

  void AStar::removeFirstFromOpenSet()
  {
    std::unique_lock<std::recursive_mutex> lock(openSetMutex);
    openSet.erase(openSet.begin());
  }

  void AStar::addToClosedSet(const Vertex& aVertex)
  {
    std::unique_lock<std::recursive_mutex> lock(closedSetMutex);
    closedSet.insert(aVertex);
  }

  void AStar::removeFromClosedSet(const Vertex& aVertex)
  {
    std::unique_lock<std::recursive_mutex> lock(closedSetMutex);
    ClosedSet::iterator i = findInClosedSet(aVertex);
    removeFromClosedSet(i);
  }

  void AStar::removeFromClosedSet(ClosedSet::iterator& i)
  {
    std::unique_lock<std::recursive_mutex> lock(closedSetMutex);
    closedSet.erase(i);
  }

  ClosedSet::iterator AStar::findInClosedSet(const Vertex& aVertex)
  {
    std::unique_lock<std::recursive_mutex> lock(closedSetMutex);
    return closedSet.find(aVertex);
  }

  ClosedSet& AStar::getCS()
  {
    std::unique_lock<std::recursive_mutex> lock(closedSetMutex);
    return closedSet;
  }

  const ClosedSet& AStar::getCS() const
  {
    std::unique_lock<std::recursive_mutex> lock(closedSetMutex);
    return closedSet;
  }

  OpenSet& AStar::getOS()
  {
    std::unique_lock<std::recursive_mutex> lock(openSetMutex);
    return openSet;
  }

  const OpenSet& AStar::getOS() const
  {
    std::unique_lock<std::recursive_mutex> lock(openSetMutex);
    return openSet;
  }

  VertexMap& AStar::getPM()
  {
    std::unique_lock<std::recursive_mutex> lock(predecessorMapMutex);
    return predecessorMap;
  }

  const VertexMap& AStar::getPM() const
  {
    std::unique_lock<std::recursive_mutex> lock(predecessorMapMutex);
    return predecessorMap;
  }
} // namespace astar