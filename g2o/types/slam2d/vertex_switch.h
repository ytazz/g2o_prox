// This class was made by Kotaro Wada. (211t373t[at]stu.kobe-u.ac.jp)

#ifndef G2O_SWITCHING_H
#define G2O_SWITCHING_H

#include "g2o/core/base_ternary_edge.h"
#include "parameter_switch_weight.h"

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

    class G2O_TYPES_SLAM2D_API VertexSwitch : public BaseVertex<1, number_t>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            VertexSwitch();

        virtual void setToOriginImpl() {
            //for (int i = 0; i < prox_maxN; i++)
            //    _estimate[i] = 1.;
            _estimate = 1.;
        }

        virtual void oplusImpl(const number_t* update);

        virtual bool setEstimateDataImpl(const number_t* est) {
            _estimate = est[0];
            return true;
        }

        virtual bool getEstimateData(number_t* est) const {
            est[0] = _estimate;
            return true;
        }

        virtual int estimateDimension() const { return 1; }

        virtual bool setMinimalEstimateDataImpl(const number_t* est) {
            return setEstimateData(est);
        }

        virtual bool getMinimalEstimateData(number_t* est) const {
            return getEstimateData(est);
        }

        virtual int minimalEstimateDimension() const { return 1; }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

    private:
    };

} // end namespace

#endif
