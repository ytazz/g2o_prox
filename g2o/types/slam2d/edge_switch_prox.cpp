// This class was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#include "edge_switch_prox.h"
#include <iostream>
#include <iomanip>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {
  using namespace std;


  // point to camera projection, monocular
  EdgeSwitchProx::EdgeSwitchProx()
      : BaseTernaryEdge < 3, std::array<int, 2>, VertexProx, VertexProx, VertexSwitch >() {
      _cacheFrom = 0;
      _cacheTo = 0;
    information().setZero();
  }

  bool EdgeSwitchProx::resolveCaches() {
      ParameterVector pv(0);
      resolveCache(_cacheFrom, (OptimizableGraph::Vertex*)_vertices[0],
          "CACHE_PROX", pv);
      resolveCache(_cacheTo, (OptimizableGraph::Vertex*)_vertices[1],
          "CACHE_PROX", pv);
      return (_cacheFrom && _cacheTo);
  }

  bool EdgeSwitchProx::read(std::istream& is) {
    setMeasurementData(0);
    if (is.bad()) return false;
    information().setZero();

    is >> _measurement[0] >> _measurement[1];
    double info, sinfo;
    is >> info >> sinfo;
    information()(0, 0) = info;
    information()(1, 1) = info;
    information()(2, 2) = sinfo;
    if (is.bad()) return false;
    return true;
  }

  bool EdgeSwitchProx::write(std::ostream& os) const {
      os << _measurement[0] << " " << _measurement[1] << " "
          << information()(0, 0) << " " << information()(2, 2) << " ";
      return true;
  }

  void EdgeSwitchProx::computeError() {
      _error.setZero();
      const number_t vk = dynamic_cast<VertexSwitch*>(_vertices[2])->estimate();
      const int fi = fp(), ti = tp();
      _error(0, 0) = vk * fc()->e(fi).dot(tc()->q(ti) - fc()->q(fi));
      _error(1, 0) = vk * tc()->e(ti).dot(fc()->q(fi) - tc()->q(ti));
      _error(2, 0) = cst(1.) - vk;
  }

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void EdgeSwitchProx::linearizeOplus() {
      _jacobianOplusXi.setZero();
      _jacobianOplusXj.setZero();
      _jacobianOplusXk.setZero();
      const SE2 vi = dynamic_cast<VertexProx*>(_vertices[0])->estimate();
      const SE2 vj = dynamic_cast<VertexProx*>(_vertices[1])->estimate();
      const number_t vk = dynamic_cast<VertexSwitch*>(_vertices[2])->estimate();
      const Matrix2 Ri = vi.rotation().toRotationMatrix(),
          Rj = vj.rotation().toRotationMatrix();
      const int fi = fp(), ti = tp();
      _jacobianOplusXi.block<1, 2>(0, 0) = -vk * fc()->u(fi);
      _jacobianOplusXi(0, 2) = vk * (fc()->de(fi).dot(tc()->q(ti) - fc()->q(fi)) - fc()->e(fi).dot(fc()->dq(fi)));
      _jacobianOplusXi.block<1, 2>(1, 0) = vk * Ri.transpose() * tc()->e(ti);
      _jacobianOplusXi(1, 2) = vk * tc()->e(ti).dot(fc()->dq(fi));
      _jacobianOplusXj.block<1, 2>(0, 0) = vk * Rj.transpose() * fc()->e(fi);
      _jacobianOplusXj(0, 2) = vk * fc()->e(fi).dot(tc()->dq(ti));
      _jacobianOplusXj.block<1, 2>(1, 0) = -vk * tc()->u(ti);
      _jacobianOplusXj(1, 2) = vk * (tc()->de(ti).dot(fc()->q(fi) - tc()->q(ti)) - tc()->e(ti).dot(tc()->dq(ti)));
      _jacobianOplusXk.block<2, 1>(0, 0) << cst(fc()->e(fi).dot(tc()->q(ti) - fc()->q(fi))), cst(tc()->e(ti).dot(fc()->q(fi) - tc()->q(ti)));
      _jacobianOplusXk(2, 0) = -cst(1.);  }
#endif

  bool EdgeSwitchProx::setMeasurementFromState(){
      //VertexSE2Proximity* from_node = static_cast<VertexSE2Proximity*>(_vertices[0]);
      //VertexSE2Proximity* to_node = static_cast<VertexSE2Proximity*>(_vertices[1]);
      //_measurement = from_node->estimate().inverse() * to_node->estimate();
    return true;
  }


  void EdgeSwitchProx::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to_*/)
  {
    //VertexSE2Proximity* from_node = dynamic_cast<VertexSE2Proximity*>(_vertices[0]);
    //VertexSE2Proximity* to_node = dynamic_cast<VertexSE2Proximity*>(_vertices[1]);
    //if (from.count(from_node) > 0)
    //    to_node->setEstimate(from_node->estimate() * _measurement);
    //else
    //    from_node->setEstimate(to_node->estimate() * _measurement.inverse());
  }

  // point to camera projection, monocular
  EdgeSwitchProxSIP::EdgeSwitchProxSIP()
      : EdgeSwitchProx() {
      _ProxPairWeightInfo = 0;
      resizeParameters(1);
      installParameter(_ProxPairWeightInfo, 0);
  }

  bool EdgeSwitchProxSIP::read(std::istream& is) {
      setMeasurementData(0);
      if (is.bad()) return false;
      information().setZero();
      if (!setParameterId(0, 0))
          return false;

      is >> _measurement[0] >> _measurement[1];
      double info;
      is >> info;
      information()(0, 0) = info;
      information()(1, 1) = info;
      information()(2, 2) = _ProxPairWeightInfo->Info();
      if (is.bad()) return false;
      return true;
  }

  bool EdgeSwitchProxSIP::write(std::ostream& os) const {
      os << _measurement[0] << " " << _measurement[1] << " " << information()(0, 0) << " ";
      return true;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSwitchProxDrawAction::EdgeSwitchProxDrawAction()
      : DrawAction(typeid(EdgeSwitchProx).name()), _triangleY(nullptr), _triangleZ(nullptr) {}

  bool EdgeSwitchProxDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_) {
      if (!DrawAction::refreshPropertyPtrs(params_))
          return false;
      if (_previousParams) {
          _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::N_Edge_Width", 3.0f);
          _triangleZ = _previousParams->makeProperty<FloatProperty>(_typeName + "::P_Edge_Width", 2.0f);
      }
      else {
          _triangleY = 0;
          _triangleZ = 0;
      }
      return true;
  }

  HyperGraphElementAction* EdgeSwitchProxDrawAction::operator()(HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_) {
      if (typeid(*element).name() != _typeName)
          return nullptr;

      refreshPropertyPtrs(params_);
      if (!_previousParams)
          return this;

      if (_show && !_show->value())
          return this;

      EdgeSwitchProx* e = static_cast<EdgeSwitchProx*>(element);
      VertexProx* from = static_cast<VertexProx*>(e->vertex(0));
      VertexProx* to = static_cast<VertexProx*>(e->vertex(1));
      if (!from && !to)
          return this;
      SE2 fromTransform;
      SE2 toTransform;
      Vector2 fromProximity;
      Vector2 toProximity;
      glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
      glDisable(GL_LIGHTING);
      fromTransform = from->estimate();
      toTransform = to->estimate();
      fromProximity = e->fc()->q(e->fp());
      toProximity = e->tc()->q(e->tp());
      if (_triangleY->value() != 0) {
          glLineWidth((float)_triangleY->value());
          glColor3f(POSE_EDGE_COLOR);
          glBegin(GL_LINES);
          glVertex3f((float)fromTransform.translation().x(), (float)fromTransform.translation().y(), 0.f);
          glVertex3f((float)toTransform.translation().x(), (float)toTransform.translation().y(), 0.f);
          glEnd();
      }
      if (_triangleZ->value() != 0) {
          glLineWidth((float)_triangleZ->value());
          glColor3f(LANDMARK_EDGE_COLOR);
          glBegin(GL_LINES);
          glVertex3f((float)fromProximity.x(), (float)fromProximity.y(), 0.f);
          glVertex3f((float)toProximity.x(), (float)toProximity.y(), 0.f);
          glEnd();
      }
      glPopAttrib();
      return this;
  }
#endif
}
