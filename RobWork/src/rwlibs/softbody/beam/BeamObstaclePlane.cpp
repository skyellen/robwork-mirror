/*
Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#include "BeamObstaclePlane.hpp"

#include <rw/math/EAA.hpp>

using namespace rw::math;
using namespace rwlibs::softbody;

BeamObstaclePlane::BeamObstaclePlane ( const rw::geometry::Plane& plane, const rw::math::Transform3D< double >& trans ) :
    _planePtr(new rw::geometry::Plane(plane)),
    _trans(trans),
    _geomPtr(new rw::geometry::Geometry(_planePtr) )
{

}


Transform3D< double > BeamObstaclePlane::getTransform ( void ) const {
    return _trans;
}



void BeamObstaclePlane::setTransform ( const Transform3D< double >& trans ) {
    _trans = trans;
}







double BeamObstaclePlane::get_thetaTCP ( const rw::math::Transform3D< double >& planeTbeam ) const {
    EAA<> eaa(planeTbeam.R());
    
    return eaa[2];
}



double BeamObstaclePlane::get_yTCP ( const rw::math::Transform3D< double >& planeTbeam ) const {
    return planeTbeam.P()[1] * 1.0e3;
}



rw::math::Transform3D< double > BeamObstaclePlane::compute_planeTbeam ( const rw::math::Transform3D< double >& Tbeam ) {
    Transform3D<> planeTbeam = _trans; // init to Tplane
    
    // compute beam to plane transform    
    Transform3D<>::invMult(planeTbeam, Tbeam);
    
    planeTbeam.R().normalize();
    
    return planeTbeam;
}



rw::geometry::Plane::Ptr BeamObstaclePlane::getPlane ( void ) const {
    return _planePtr;
}



rw::geometry::Geometry::Ptr BeamObstaclePlane::getPlaneGeometry ( void ) const {
    return _geomPtr;
}
