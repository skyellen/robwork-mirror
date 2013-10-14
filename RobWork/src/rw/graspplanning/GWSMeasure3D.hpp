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


#ifndef RW_GRASPPLANNING_SANDBOX_GWSMeasure3D_HPP_
#define RW_GRASPPLANNING_SANDBOX_GWSMeasure3D_HPP_


#include <rw/math/Vector3D.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/graspplanning/GraspQualityMeasure3D.hpp>
#include <rw/sensor/Contact3D.hpp>
#include <rw/geometry/ConvexHull3D.hpp>
#include <rw/geometry/QHullND.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief Represents the grasp wrench space as a 3D force space and a
 * 3D torque space.
 *
 *
 */
class GWSMeasure3D: public GraspQualityMeasure3D {
public:
    /**
     * @brief constructor
     * @param chull [in] a convex hull factory
     * @param resolution [in] the number of vertices to use in the approximation
     * of the friction cone.
     * @param useUnitVectors [in] true if the unit wrench should be used, false otherwise
     */
    GWSMeasure3D(int resolution=8, bool useUnitVectors=false);

	/**
	 * @brief destructor
	 */
    virtual ~GWSMeasure3D(){};

    //// inherited from quality measure
    double quality(const rw::graspplanning::Grasp3D& grasp) const;

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
    bool isGraspStable(){ return _isInside;};

    /**
     * @brief get the minimum force that will break the grasp
     * 
     * Returns the minimum distance from the cener of the object to the wall of the wrench space.
     * 
     * @return minimum breaking force
     */
    double getMinWrench(){ return _minWrench; };

	/**
	 * @brief get the force that will break the grasp
	 * 
	 * Returns the minimum force neccesary to break the grasp for the best case scenario -
	 * if the center of the object coincides with the center of the wrench space.
	 * 
	 * @return minimum breaking force
	 */
    double getAvgWrench(){ return _avgWrench; };
    
    /**
     * @brief get average wrench in relation to the origin
     */
    double getAverageOriginWrench() { return _avgOriginWrench; }
    
    /**
     * @brief get average wrench in relation to center of the convex hull
     */
    double getAverageCenterWrench() { return _avgCenterWrench; }

    void setLambda(double lambda){ _lambda = lambda;}

private:
    rw::common::Ptr<rw::geometry::QHullND<6> > _chullCalculator;
    rw::math::Vector3D<> _objCenter;
    int _resolution;

    mutable bool _isInside;
    mutable double _minWrench, _avgWrench, _avgCenterWrench, _avgOriginWrench;
    bool _useUnitVectors;

    double _lambda;
};


}
}

#endif /* FORCECLOSURE3D_HPP_ */
