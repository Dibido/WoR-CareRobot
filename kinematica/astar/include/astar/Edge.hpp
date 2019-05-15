#ifndef EDGE_HPP
#define EDGE_HPP

#include "Vertex.hpp"
#include <iostream>
#include <set>

namespace astar
{
  struct Edge
  {
    Edge(const Vertex& aVertex1, const Vertex& aVertex2)
        : vertex1(aVertex1), vertex2(aVertex2)
    {
    }
    Edge(const Edge& anEdge) : vertex1(anEdge.vertex1), vertex2(anEdge.vertex2)
    {
    }

    const Vertex& thisSide(const Vertex& aVertex) const
    {
      if (vertex1.equalPoint(aVertex))
        return vertex1;
      if (vertex2.equalPoint(aVertex))
        return vertex2;
      throw std::logic_error("thisSide: huh???");
    }

    const Vertex& otherSide(const Vertex& aVertex) const
    {
      if (vertex1.equalPoint(aVertex))
        return vertex2;
      if (vertex2.equalPoint(aVertex))
        return vertex1;
      throw std::logic_error("otherSide: huh???");
    }

    Vertex vertex1;
    Vertex vertex2;
  }; // struct Edge

  /**
   *
   * @param os
   * @param anEdge
   * @return
   */
  inline std::ostream& operator<<(std::ostream& os, const Edge& anEdge)
  {
    return os << anEdge.vertex1 << " -> " << anEdge.vertex2;
  }
} // namespace astar
#endif // EDGE_HPP