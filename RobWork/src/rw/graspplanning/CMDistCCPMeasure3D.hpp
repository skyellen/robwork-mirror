#ifndef CMDISTCCPMEASURE_HPP_
#define CMDISTCCPMEASURE_HPP_

#include "Grasp3D.hpp"
#include "GraspQualityMeasure3D.hpp"
#include <rw/geometry/TriMesh.hpp>
#include <vector>

namespace rw {
namespace graspplanning {


/**
 * @brief computes the quality as a function of the distance between
 * the center of mass (COM) and the Center of the Contact Polygon (CCP)
 *
 * See ROA 3.2.3
 */
class CMDistCCPMeasure3D : public GraspQualityMeasure3D {
public:
    /**
     * @brief constructor
     * @param CM [in] The center of mass of the object described in the grasp frame.
     * @param maxDist [in] The max dist between CM and CCP that is allowed.
     * @return
     */
    CMDistCCPMeasure3D(const rw::math::Vector3D<>& CM, double maxDist):
        _CM(CM),_maxDist(maxDist)
    {}

    /**
     * @brief destructor
     */
    virtual ~CMDistCCPMeasure3D(){};

    /**
     * @brief computes the quality of the grasp such that the quality
     * is in the interval [0;1] with 1 being the highest quality.
     */
    virtual double quality(const Grasp3D& grasp) const;

private:
    rw::math::Vector3D<> _CM;
    double _maxDist;
};
}
}

#endif /*QUALITYMEASURE_HPP_*/
