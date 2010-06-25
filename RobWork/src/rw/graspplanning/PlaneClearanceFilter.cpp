/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#include "PlaneClearanceFilter.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Constants.hpp>
#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::graspplanning;
using namespace rw::sensor;


bool PlaneClearanceFilter::isValid(const Grasp3D& grasp){
    // 1. check the distance from plane to all contacts
    //std::cout << "Check clearance " << std::endl;
    BOOST_FOREACH(const Contact3D &con, grasp.contacts){
        // transform the position to the plane frame
        Vector3D<> p = inverse(_planeFrame) * con.p;
        //std::cout << p << std::endl;
        if(p[2]<_clearance){
            return false;
        }
    }

    //std::cout << "check angle " << std::endl;
    // 2. check that the angle between approach vector and plane is not too large
    BOOST_FOREACH(const Contact3D &con, grasp.contacts){
        // transform the normal vector to the plane frame
        Vector3D<> n = inverse(_planeFrame.R()) * con.n;
        // calculate the angle between normal and plane, where a n with (0,0,-1)
        // has an min angle of -90 degree and (0,0,1) has an max angle of 90 degree.
        double ang = angle( Vector3D<>(0,0,-1), n); // =
        if(ang<_minAngle+Pi/2)
            return false;
    }

    return true;
}

bool PlaneClearanceFilter::isValid(const Contact3D& con){
    Vector3D<> p = inverse(_planeFrame) * con.p;
    //std::cout << p << std::endl;
    if(p[2]<_clearance){
        return false;
    }

    // transform the normal vector to the plane frame
    Vector3D<> n = inverse(_planeFrame.R()) * con.n;
    // calculate the angle between normal and plane, where a n with (0,0,-1)
    // has an min angle of -90 degree and (0,0,1) has an max angle of 90 degree.
    double ang = angle( Vector3D<>(0,0,-1), n); // =
    std::cout << "ANGLE: " << ang*Rad2Deg << " < " << (_minAngle+Pi/2)*Rad2Deg << std::endl;
    if(ang<_minAngle+Pi/2)
        return false;

    return true;
}
