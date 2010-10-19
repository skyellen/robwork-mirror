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

#include "WrenchMeasure3D.hpp"

#include <rw/math/EAA.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Constants.hpp>
#include <rw/geometry/GiftWrapHull3D.hpp>


#include <boost/foreach.hpp>
#include <iostream>

using namespace rw::math;
using namespace rw::geometry;
using namespace rw::graspplanning;


namespace {

    /**
     * calculate the normalized friction cone from a force and friction
     * @param force
     * @param mu
     * @return
     */
    std::vector<Vector3D<> > getCone(const Vector3D<>& normal, double nforce, double mu, int resolution){
        Vector3D<> unormal = normalize(normal);
        //double angle = atan(mu);
        //tan(angle)*1
        // create a vector that is tangent to the normal
        const double EPSILON = 0.000001;
        Vector3D<> tdir;
        if( fabs(normal(0))<EPSILON && fabs(normal(1))<EPSILON ){
            tdir = normalize(  Vector3D<>(0,-normal(2),normal(1)) );
        } else {
            tdir = normalize(  Vector3D<>(-normal(1),normal(0),0) );
        }
        Vector3D<> tdirScaled = tdir * (nforce * mu);

        Vector3D<> fnormal = unormal * nforce;
        std::vector<Vector3D<> > coneVerts;
        // rotate the vector around the normal in some fixed steps
        Vector3D<> P = tdirScaled; // vector to rotate
        Vector3D<> A = unormal; // vector to rotate about

        EAA<> eaa(unormal, 2*Pi/resolution );
        Rotation3D<> rot( eaa.toRotation3D() );
        for(int i=0; i<resolution-1; i++){
            coneVerts.push_back(fnormal+P);
            P = rot*P;
        }
        return coneVerts;
    }

}

WrenchMeasure3D::WrenchMeasure3D(int resolution, bool useUnitVectors):
    _chullCalculator( rw::common::ownedPtr(new GiftWrapHull3D() ) ),
    _resolution(resolution),
    _useUnitVectors(useUnitVectors)
{
}


double WrenchMeasure3D::quality(const Grasp3D& grasp) const {
    std::vector< Vector3D<> > fvertices;
    std::vector< Vector3D<> > tvertices;

    BOOST_FOREACH(const rw::sensor::Contact3D& c, grasp.contacts ){
        // std::cout  << "get cone: " << c.n << " " << c.normalForce << std::endl;
         if(c.normalForce<0.0001){
        	 RW_WARN("Normal force too small! : " << c.normalForce);
        	 continue;
         }

    	 Vector3D<> arm = c.p - _objCenter;
    	 double normalForce = 1.0;
    	 if(!_useUnitVectors)
    	     normalForce = c.normalForce;
         std::vector<Vector3D<> > verts = getCone(c.n,normalForce,c.mu,_resolution);
         BOOST_FOREACH(const Vector3D<> &force, verts){
             fvertices.push_back( force );
             tvertices.push_back( cross(arm,force) );
         }
    }

    // first do the force space
    _chullCalculator->rebuild( fvertices );
    // test if the center is inside
    _isForceInside = _chullCalculator->isInside( Vector3D<>(0,0,0) );
    _minForce = _chullCalculator->getMinDist( Vector3D<>(0,0,0) );

    // first do the force space
    _chullCalculator->rebuild( tvertices );
    // test if the center is inside

    _isTorqueInside = _chullCalculator->isInside( Vector3D<>(0,0,0) );
    _minTorque = _chullCalculator->getMinDist( Vector3D<>(0,0,0) );
    if( _isTorqueInside && _isForceInside ){
        return (_minForce + _minTorque)/2;
    }
    return 0;
}
