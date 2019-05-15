#ifndef VERTEX_HPP
#define VERTEX_HPP
#include <iostream>
namespace astar
{

  struct Vertex
  {
    /**
     *
     */
    Vertex(unsigned long anX, unsigned long anY, unsigned long anZ)
        : x(anX), y(anY), z(anZ), actualCost(0.0), heuristicCost(0.0)
    {
    }
    /**
     *
     */
    bool lessCost(const Vertex& aVertex) const
    {
      if (heuristicCost < aVertex.heuristicCost)
        return true;
      // less uncertainty if the actual cost is smaller
      if (heuristicCost == aVertex.heuristicCost)
        return actualCost > aVertex.actualCost;
      return false;
    }
    /**
     *
     */
    bool lessId(const Vertex& aVertex) const
    {
      if (x != aVertex.x)
        return x < aVertex.x;
      if (y != aVertex.y)
        return y < aVertex.y;
      return z < aVertex.z;
    }
    /**
     *
     */
    bool equalPoint(const Vertex& aVertex) const
    {
      return x == aVertex.x && y == aVertex.y && z == aVertex.z;
    }

    long x;
    long y;
    long z;

    double actualCost;
    double heuristicCost;
  };
  // struct Vertex

  /**
   *
   */
  struct VertexLessCostCompare
  {
    bool operator()(const Vertex& lhs, const Vertex& rhs) const
    {
      return lhs.lessCost(rhs);
    }
  };
  // struct VertexCostCompare
  /**
   *
   */
  struct VertexLessIdCompare
  {
    bool operator()(const Vertex& lhs, const Vertex& rhs) const
    {
      return lhs.lessId(rhs);
    }
  };
  // struct VertexIdCompare
  /**
   *
   */
  struct VertexEqualPointCompare
  {
    bool operator()(const Vertex& lhs, const Vertex& rhs) const
    {
      return lhs.equalPoint(rhs);
    }
  };

  /**
   *
   * @param os
   * @param aVertex
   * @return
   */
  inline std::ostream& operator<<(std::ostream& os, const Vertex& aVertex)
  {
    return os << "(" << aVertex.x << "," << aVertex.y << "," << aVertex.z
              << "), " << aVertex.actualCost << " " << aVertex.heuristicCost;
  }

} // namespace astar

#endif // VERTEX_HPP