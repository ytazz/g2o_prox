// This library was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#ifndef G2O_VERTEX_SE2_PROXIMITY_H
#define G2O_VERTEX_SE2_PROXIMITY_H

#include "g2o/core/base_binary_edge.h"

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

  class G2O_TYPES_SLAM2D_API VertexProx : public BaseVertex<3, SE2>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          VertexProx();

      virtual void setToOriginImpl() {
        _estimate = SE2();
      }

      virtual void oplusImpl(const number_t* update);

      virtual bool setEstimateDataImpl(const number_t* est){
        _estimate=SE2(est[0], est[1], est[2]);
        return true;
      }

      virtual bool getEstimateData(number_t* est) const {
        Eigen::Map<Vector3> v(est);
        v = _estimate.toVector();
        return true;
      }
      
      virtual int estimateDimension() const { return 3; }

      virtual bool setMinimalEstimateDataImpl(const number_t* est){
        return setEstimateData(est);
      }

      virtual bool getMinimalEstimateData(number_t* est) const {
        return getEstimateData(est);
      }
      
      virtual int minimalEstimateDimension() const { return 3; }

      bool setProximityParam(const std::vector<Vector2>& pp);

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual inline int numProx() const { return _proximityParam.size(); };
      virtual inline Vector2 proximity(int i) const {
          if(0 <= i && i < numProx())
            return _proximityParam[i];
          std::cerr << typeid(*this).name() << " : Out of Proximity Index Range!" << std::endl;
          return Vector2();
      };

  private:
      std::vector<Vector2> _proximityParam;
  };

  class G2O_TYPES_SLAM2D_API VertexProxWriteGnuplotAction: public WriteGnuplotAction {
  public:
      VertexProxWriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API VertexProxDrawAction: public DrawAction{
  public:
    VertexProxDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  protected:
    HyperGraphElementAction* _drawActions;
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _arrow_length, * _arrow_width, * _point_size, * _line_width;

  };
#endif

} // end namespace

#endif
