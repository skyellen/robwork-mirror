#ifndef APPROACHMEASURE_HPP_
#define APPROACHMEASURE_HPP_

#include "Grasp3D.hpp"
#include "GraspQualityMeasure3D.hpp"

#include <vector>

/**
 * @brief computes the quality as a function of the angle between
 * the approach

 */
namespace rw {
namespace graspplanning {

class ApproachMeasure3D : public GraspQualityMeasure3D {
public:
    /**
     * @brief constructor
     * @param maxAngle [in] the maximum allowed angle between approach and
     * surface normal. Grasps violating this will recieve high penalty on
     * quality.
     * @return
     */
    ApproachMeasure3D(double maxAngle):
        _maxAngle(maxAngle)
    {}

    /**
     * @brief destructor
     */
    virtual ~ApproachMeasure3D(){};

    /**
     * @brief computes the quality of the grasp such that the quality
     * is in the interval [0;1] with 1 being the highest quality.
     */
    virtual double quality(const Grasp3D& grasp) const;

private:

    double _maxAngle;
};

}
}

#endif /*QUALITYMEASURE_HPP_*/
