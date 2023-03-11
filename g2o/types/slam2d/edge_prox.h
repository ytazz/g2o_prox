// This library was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#ifndef G2O_EDGE_SE2_PROXIMITY_H_
#define G2O_EDGE_SE2_PROXIMITY_H_

//#define NUMERIC_JACOBIAN_TWO_D_TYPES

#include "g2o/core/base_binary_edge.h"
#include "vertex_prox.h"
#include "cache_prox.h"
#include "g2o_types_slam2d_api.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/robust_kernel_factory.h"

namespace g2o {

class G2O_TYPES_SLAM2D_API EdgeProx : public BaseBinaryEdge < 2, std::array<int, 2>, VertexProx, VertexProx > {
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          EdgeProx();

      virtual bool resolveCaches();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      virtual void linearizeOplus();
#endif

      virtual void setMeasurement(const std::array<int, 2> m) {
          for (int i = 0; i < 2; i++)
              _measurement[i] = m[i];
      }

      virtual void setMeasurement(const int m[2]) {
          for (int i = 0; i < 2; i++)
              _measurement[i] = m[i];
      }

      virtual bool setMeasurementData(const int* d) {
          for(int i = 0; i < 2; i++)
              _measurement[i] = d[i];
          return true;
      }

      virtual bool setMeasurementData(const int d) {
          for (int i = 0; i < 2; i++)
              _measurement[i] = d;
          return true;
      }

      virtual bool getMeasurementData(int d[2]) const {
          for (int i = 0; i < 2; i++)
              d[i] = _measurement[i];
          return true;
      }

      virtual int measurementDimension() const { return 3; }

      virtual bool setMeasurementFromState();

      virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
          (void)to;
          return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);
      }

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

      const virtual inline int fp() const { return _measurement[0]; };
      const virtual inline int tp() const { return _measurement[1]; };
      const virtual inline CacheProximity* fc() const { return _cacheFrom; };
      const virtual inline CacheProximity* tc() const { return _cacheTo; };

protected:
    CacheProximity* _cacheFrom, * _cacheTo;
};

class G2O_TYPES_SLAM2D_API EdgeMEstProx : public EdgeProx {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeMEstProx() : EdgeProx(), _strRobustKernel(){};

    virtual void setStrRobustKernel(const std::string& strRobustKernel) { _strRobustKernel = strRobustKernel; };

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

protected:
    std::string _strRobustKernel;
};

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API EdgeProxDrawAction : public DrawAction {
  public:
      EdgeProxDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
          HyperGraphElementAction::Parameters* params_);
  protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _triangleY, * _triangleZ;
  };

  class G2O_TYPES_SLAM2D_API EdgeMEstProxDrawAction : public DrawAction {
  public:
      EdgeMEstProxDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
          HyperGraphElementAction::Parameters* params_);
  protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _triangleY, * _triangleZ;
  };
#endif

}
#endif
