// This library was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#ifndef G2O_PARAMETER_PROXIMITY_H_
#define G2O_PARAMETER_PROXIMITY_H_

#include <cmath>
#include <vector>

#include "g2o/core/cache.h"
#include "g2o_types_slam2d_api.h"
#include "vertex_prox.h"

namespace g2o {

class G2O_TYPES_SLAM2D_API CacheProximity : public Cache {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CacheProximity();
  virtual void updateImpl();

  inline const int  numProx() const { return _locProxPoints.size(); }
  inline const Vector2& r(int i) const { return _locProxPoints[i]; }
  inline const Vector2& u(int i) const { return _locNormalVecs[i]; }
  inline const Vector2& q(int i) const { return _absProxPoints[i]; }
  inline const Vector2& e(int i) const { return _absNormalVecs[i]; }
  inline const Vector2& dq(int i) const { return _diffProxPoints[i]; }
  inline const Vector2& de(int i) const { return _diffNormalVecs[i]; }

 protected:
     std::vector<Vector2, Eigen::aligned_allocator<Vector2> > _locProxPoints;
     std::vector<Vector2, Eigen::aligned_allocator<Vector2> > _absProxPoints;
     std::vector<Vector2, Eigen::aligned_allocator<Vector2> > _diffProxPoints;
     std::vector<Vector2, Eigen::aligned_allocator<Vector2> > _locNormalVecs;
     std::vector<Vector2, Eigen::aligned_allocator<Vector2> > _absNormalVecs;
     std::vector<Vector2, Eigen::aligned_allocator<Vector2> > _diffNormalVecs;

 protected:
  virtual bool resolveDependencies();
};

}
#endif
