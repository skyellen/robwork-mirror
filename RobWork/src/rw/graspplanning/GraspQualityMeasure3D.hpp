#ifndef GRASPQUALITYMEASURE3D_HPP_
#define GRASPQUALITYMEASURE3D_HPP_

#include "Grasp3D.hpp"

namespace rw {
namespace graspplanning {

/**
 *
 */
class GraspQualityMeasure3D {
public:

    /**
     * @brief computes the quality of the grasp such that the quality
     * is in the interval [0;1] with 1 being the highest quality.
     */
    virtual double quality(const Grasp3D& grasp) const = 0;

};

}
}

#endif /*GRASPQUALITYMEASURE3D_HPP_*/
