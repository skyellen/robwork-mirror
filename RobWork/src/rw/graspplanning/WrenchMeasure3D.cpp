#include "WrenchMeasure3D.hpp"
/*
 * WrenchMeasure3D.cpp
 *
 *  Created on: 20-07-2008
 *      Author: jimali
 */

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
    std::vector<Vector3D<> > getNormalizedCone(const Vector3D<>& normal, double nforce, double mu, int resolution){
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

/*
WrenchMeasure3D* WrenchMeasure3D::calcWrenchSpace(
        std::vector<Contact3D> contacts,
        const Vector3D<>& geomcenter,
        int res){

    typedef std::vector<Vector3D<> > arrayType;
    std::vector< Vector3D<> > fvertices;
    std::vector< Vector3D<> > tvertices;
    // first calculate the Force space as the convex hull of all force cones
    // and while we are at it calculate the torque space, which is calculated using the discretised force cones
    BOOST_FOREACH(Contact3D& c, contacts ){
       // std::cout  << "get cone: " << c.n << " " << c.normalForce << std::endl;
        Vector3D<> arm = c.p - geomcenter;
        std::vector<Vector3D<> > verts = getNormalizedCone(c.n,1,c.mu,res);
        BOOST_FOREACH(const Vector3D<> &force, verts){
            fvertices.push_back( force );
            tvertices.push_back( cross(arm,force) );
        }
    }


    GiftWrapHull3D *forceHull = new GiftWrapHull3D();
    forceHull->rebuild(fvertices);

    GiftWrapHull3D *torqueHull = new GiftWrapHull3D();
    torqueHull->rebuild(tvertices);

    return new WrenchMeasure3D(forceHull,torqueHull);
}
*/

double WrenchMeasure3D::quality(const Grasp3D& grasp) const {
    std::vector< Vector3D<> > fvertices;
    std::vector< Vector3D<> > tvertices;

    BOOST_FOREACH(const rw::sensor::Contact3D& c, grasp.contacts ){
        // std::cout  << "get cone: " << c.n << " " << c.normalForce << std::endl;
         Vector3D<> arm = c.p - _objCenter;
         std::vector<Vector3D<> > verts = getNormalizedCone(c.n,c.normalForce,c.mu,_resolution);
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
