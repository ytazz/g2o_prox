// This class was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#include <string>
#include "vertex_switch.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

    VertexSwitch::VertexSwitch() :
        BaseVertex<1, number_t>() { 
        setToOriginImpl();
    }
  void VertexSwitch::oplusImpl(const number_t* update) {
      _estimate += update[0];
  }

  bool VertexSwitch::read(std::istream& is) {
      setToOriginImpl();
    if (is.bad()) return true;
    is >> _estimate;
    return true;
  }

  bool VertexSwitch::write(std::ostream& os) const {
    os << _estimate << " ";
    return true;
  }

} // end namespace
