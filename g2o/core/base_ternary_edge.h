// This class was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#ifndef G2O_BASE_TERNARY_EDGE_H
#define G2O_BASE_TERNARY_EDGE_H

#include "base_fixed_sized_edge.h"

namespace g2o {

// This could be a simple using statement, but in multiple places
// _jacobianOplusXi, _jacobianOplusXj and _jacobianOplusXk are used.
template <int D, typename E, typename VertexXi, typename VertexXj, typename VertexXk>
class BaseTernaryEdge : public BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk> {
 public:
  using VertexXiType = VertexXi;
  using VertexXjType = VertexXj;
  using VertexXkType = VertexXk;
  BaseTernaryEdge() : BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk>(){};

 protected:
  typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk>::template JacobianType<
      D, VertexXi::Dimension>& _jacobianOplusXi = std::get<0>(this->_jacobianOplus);
  typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk>::template JacobianType<
      D, VertexXj::Dimension>& _jacobianOplusXj = std::get<1>(this->_jacobianOplus);
  typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk>::template JacobianType<
      D, VertexXk::Dimension>& _jacobianOplusXk = std::get<2>(this->_jacobianOplus);
};

}  // namespace g2o

#endif
