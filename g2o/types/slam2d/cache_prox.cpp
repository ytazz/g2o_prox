// This library was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#include "cache_prox.h"
#include <iostream>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

CacheProximity::CacheProximity() : Cache(){}

bool CacheProximity::resolveDependencies() {
    return true;
}

void CacheProximity::updateImpl() {
  const VertexProx* v = static_cast<const VertexProx*>(vertex());
  Eigen::Rotation2Dd dR = Eigen::Rotation2Dd(v->estimate().rotation().angle() + M_PI_2);
  int n = v->numProx();
  _locProxPoints.resize(n);
  _locNormalVecs.resize(n);
  _absProxPoints.resize(n);
  _absNormalVecs.resize(n);
  _diffProxPoints.resize(n);
  _diffNormalVecs.resize(n);
  for (int i = 0; i < n; i++) {
      _locProxPoints[i] = v->proximity(i);
      _locNormalVecs[i] = r(i) / r(i).norm();
      _absProxPoints[i] = v->estimate() * r(i);
      _absNormalVecs[i] = v->estimate().rotation() * u(i);
      _diffProxPoints[i] = dR * r(i);
      _diffNormalVecs[i] = dR * u(i);
  }
}

}
