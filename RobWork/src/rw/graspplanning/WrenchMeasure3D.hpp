/*
 * WrenchMeasure3D.hpp
 *
 *  Created on: 20-07-2008
 *      Author: jimali
 */

#ifndef FORCECLOSURE3D_HPP_
#define FORCECLOSURE3D_HPP_


#include <rw/math/Vector3D.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include "GraspQualityMeasure3D.hpp"
#include <rw/sensor/Contact3D.hpp>
#include <rw/geometry/ConvexHull3D.hpp>


namespace rw {
namespace graspplanning {

/**
 * @brief Represents the grasp wrench space as a 3D force space and a
 * 3D torque space.
 *
 *
 */
class WrenchMeasure3D: public GraspQualityMeasure3D {
public:
    /**
     * @brief constructor, takes ownership of convex hull calculator
     * @param chull
     * @param resolution [in] the number of vertices to use in the approximation
     * of the friction cone.
     * @return
     */
    WrenchMeasure3D(rw::geometry::ConvexHull3D* chull, int resolution):
        _chullCalculator( rw::common::ownedPtr(chull) ), _resolution(resolution)
    {}


	/**
	 * @brief destructor
	 */
    virtual ~WrenchMeasure3D(){};

    //// inherited from quality measure
    double quality(const Grasp3D& grasp) const;

    void setObjectCenter( const rw::math::Vector3D<>& center){ _objCenter = center; };

    bool isGraspStable(){ return _isForceInside && _isTorqueInside;};

    double getMinForce(){ return _minForce; };

    double getMinTorque(){ return _minTorque; };

private:
    rw::common::Ptr<rw::geometry::ConvexHull3D> _chullCalculator;
    rw::math::Vector3D<> _objCenter;
    int _resolution;

    mutable bool _isForceInside,_isTorqueInside;
    mutable double _minForce,_minTorque;
};

}
}

#endif /* FORCECLOSURE3D_HPP_ */
