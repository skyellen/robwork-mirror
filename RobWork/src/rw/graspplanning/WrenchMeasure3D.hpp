/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_GRASPPLANNING_WRENCHMEASURE3D_HPP_
#define RW_GRASPPLANNING_WRENCHMEASURE3D_HPP_


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
     * @brief constructor
     * @param chull [in] a convex hull factory
     * @param resolution [in] the number of vertices to use in the approximation
     * of the friction cone.
     * @param useUnitVectors [in] true if the unit wrench should be used, false otherwise
     */
    WrenchMeasure3D(int resolution=8, bool useUnitVectors=false);

    /**
     * @brief constructor
     * @param chull [in] a convex hull factory
     * @param resolution [in] the number of vertices to use in the approximation
     * of the friction cone.
     * @param useUnitVectors [in] true if the unit wrench should be used, false otherwise
     */
    WrenchMeasure3D(rw::geometry::ConvexHull3D::Ptr chull, int resolution=8, bool useUnitVectors=false):
        _chullCalculator( chull ), _resolution(resolution),_useUnitVectors(useUnitVectors)
    {}

	/**
	 * @brief destructor
	 */
    virtual ~WrenchMeasure3D(){};

    //// inherited from quality measure
    double quality(const Grasp3D& grasp) const;

    //// member functions
    /**
     * @brief set the center of the object
     * @param center
     */
    void setObjectCenter( const rw::math::Vector3D<>& center){ _objCenter = center; };

    /**
     * @brief tests if center of object is inside wrench space
     * @return true if center is inside both force and torque space
     */
    bool isGraspStable(){ return _isForceInside && _isTorqueInside;};

    /**
     * @brief get the minimum force that will break the grasp
     * @return minimum breaking force
     */
    double getMinForce(){ return _minForce; };

    /**
     * @brief get the minimum torque that will break the grasp
     * @return minimum breaking torque
     */
    double getMinTorque(){ return _minTorque; };

private:
    rw::common::Ptr<rw::geometry::ConvexHull3D> _chullCalculator;
    rw::math::Vector3D<> _objCenter;
    int _resolution;

    mutable bool _isForceInside,_isTorqueInside;
    mutable double _minForce,_minTorque;
    bool _useUnitVectors;
};

}
}

#endif /* FORCECLOSURE3D_HPP_ */
