// This library was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#include <string>
#include "vertex_prox.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

  VertexProx::VertexProx() :
    BaseVertex<3, SE2>(), _proximityParam(){}

  void VertexProx::oplusImpl(const number_t* update)
  {
    SE2 t = _estimate, dt = SE2();
    dt.setTranslation(Vector2(update[0], update[1]));
    dt.setRotation(Rotation2D(normalize_theta(update[2])));
    _estimate = t * dt;
  }

  bool VertexProx::setProximityParam(const std::vector<Vector2>& pp)
  {
      _proximityParam.clear();
      for (const Vector2& _pp : pp)
          _proximityParam.push_back(_pp);
      return true;
  }

  bool VertexProx::read(std::istream& is)
  {
    Vector3 p;
    bool state = internal::readVector(is, p);
    setEstimate(p);
    int n;
    is >> n;
    _proximityParam.resize(n);
    std::vector<Vector2> pp(n);
    for (Vector2& r : pp) {
        bool state_r = internal::readVector(is, r);
        if (state_r == false) return false;
    }
    state = setProximityParam(pp) & state;
    return state;
  }

  bool VertexProx::write(std::ostream& os) const
  {
    bool state_p = internal::writeVector(os, estimate().toVector());
    os << _proximityParam.size() << " ";
    for (const Vector2& r : _proximityParam) {
        bool state_r = internal::writeVector(os, r);
        if (state_r == false) return false;
    }
    return state_p;
  }

  VertexProxWriteGnuplotAction::VertexProxWriteGnuplotAction(): WriteGnuplotAction(typeid(VertexProx).name()){}

  HyperGraphElementAction* VertexProxWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return nullptr;
    WriteGnuplotAction::Parameters* params= dynamic_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params || !params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid output stream specified" << std::endl;
      return nullptr;
    }

    VertexProx* v = dynamic_cast<VertexProx*>(element);
    *(params->os) << v->estimate().translation().x() << " " << v->estimate().translation().y()
      << " " << v->estimate().rotation().angle() << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  VertexProxDrawAction::VertexProxDrawAction()
      : DrawAction(typeid(VertexProx).name()), _drawActions(nullptr), _arrow_length(nullptr), _arrow_width(nullptr),
      _point_size(nullptr), _line_width(nullptr) {}

  bool VertexProxDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
        _arrow_length = _previousParams->makeProperty<FloatProperty>(_typeName + "::N_Vertex_Arrow_Length", .5f);
        _arrow_width = _previousParams->makeProperty<FloatProperty>(_typeName + "::N_Vertex_Arrow_Width", .3f);
        _point_size = _previousParams->makeProperty<FloatProperty>(_typeName + "::P_Vertex_Size", 5.0f);
        _line_width = _previousParams->makeProperty<FloatProperty>(_typeName + "::PN_line_width", .0f);
    } else {
        _arrow_length = 0;
        _arrow_width = 0;
        _point_size = 0;
        _line_width = 0;
    }
    return true;
  }


  HyperGraphElementAction* VertexProxDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                 HyperGraphElementAction::Parameters* params_){
   if (typeid(*element).name()!=_typeName)
      return nullptr;
    //initializeDrawActionsCache();
    refreshPropertyPtrs(params_);

    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;

    VertexProx* that = static_cast<VertexProx*>(element);

    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
    glDisable(GL_LIGHTING);
    glColor3f(POSE_VERTEX_COLOR);
    glPushMatrix();
    glTranslatef((float)that->estimate().translation().x(),(float)that->estimate().translation().y(),0.f);
    glRotatef((float)RAD2DEG(that->estimate().rotation().angle()),0.f,0.f,1.f);
    opengl::drawArrow2D((float)_arrow_length->value(), (float)_arrow_width->value(), (float)_arrow_width->value()*.5f);
    //drawCache(that->cacheContainer(), params_);
    //drawUserData(that->userData(), params_);
    glPopMatrix();
    glColor3f(LANDMARK_VERTEX_COLOR);
    for (int i = 0; i < that->numProx(); i++) {
        if ((float)_point_size->value() != 0) {
            //glColor3f(POSE_VERTEX_COLOR);
            glPushMatrix();
            glTranslatef((float)(that->estimate() * that->proximity(i)).x(), (float)(that->estimate() * that->proximity(i)).y(), 0.f);
            glRotatef((float)RAD2DEG(that->estimate().rotation().angle()), 0.f, 0.f, 1.f);
            opengl::drawPoint((float)_point_size->value());
            //drawCache(that->cacheContainer(), params_);
            //drawUserData(that->userData(), params_);
            glPopMatrix();
            glColor3f(LANDMARK_VERTEX_COLOR);
        }
        if ((float)_line_width->value() != 0) {
            glColor3f(POSE_VERTEX_COLOR);
            glPushMatrix();
            glTranslatef((float)that->estimate().translation().x(), (float)that->estimate().translation().y(), 0.f);
            glRotatef((float)RAD2DEG(that->estimate().rotation().angle() + atan2(that->proximity(i).y(), that->proximity(i).x())), 0.f, 0.f, 1.f);
            opengl::drawLine((float)that->proximity(i).norm(), (float)_line_width->value());
            //drawCache(that->cacheContainer(), params_);
            //drawUserData(that->userData(), params_);
            glPopMatrix();
            glColor3f(LANDMARK_VERTEX_COLOR);
            //glLineWidth((float)_line_width->value());
            //glBegin(GL_LINES);
            //glVertex3f((float)that->estimate().translation().x(), (float)that->estimate().translation().y(), 0.f);
            //glVertex3f((float)(that->estimate() * that->proximity(i)).x(), (float)(that->estimate() * that->proximity(i)).y(), 0.f);
            //glEnd();
        }
    }
    //drawCache(that->cacheContainer(), params_);
    //drawUserData(that->userData(), params_);
    glPopAttrib();
    return this;
  }
#endif


} // end namespace
