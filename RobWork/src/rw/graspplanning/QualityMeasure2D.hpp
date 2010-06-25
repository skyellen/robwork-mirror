#ifndef RW_GRASPPLANNING_QUALITYMEASURE_HPP_
#define RW_GRASPPLANNING_QUALITYMEASURE_HPP_

#include "Grasp2D.hpp"

namespace rw {
namespace graspplanning {

class QualityMeasure2D {
public:

    /**
     * @compute the quality of a 2d grasp
     */
    virtual double computeQuality(const Grasp2D& grasp) const = 0;

};

}
}

#endif /*QUALITYMEASURE_HPP_*/
