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

#include "BeamGeometryCuboid.hpp"

#include <math.h>

#include "rwlibs/softbody/numerics/TrapMethod.hpp"


using namespace std;
using namespace rwlibs::softbody;


BeamGeometryCuboid::BeamGeometryCuboid (
    double dx,
    double dy,
    double dz,
    const std::vector<double> &Exvec,
    const std::vector<double> &vxvec,
    const std::vector<double> &rhovec,
    const rw::math::Transform3D<> &wTb,
    const rw::math::Vector3D<> &G
)
    :
    BeamGeometry ( dx, Exvec, vxvec, rhovec, wTb, G ),
    _H ( dz ),
    _K ( dy ) 
{
    _B0vec.resize ( getM() );

    // cache the values of B0 for all points on the beam
    for ( int i = 0; i < getM(); i++ ) {
        if ( 0 == i )
            _B0vec[i] = B0m ( getM()-1 );
        else
            _B0vec[i] = B0_fnc ( i );
    }
};



BeamGeometryCuboid::~BeamGeometryCuboid() {
}





double BeamGeometryCuboid::b0 ( const int i ) const {
    // b0 integral evaluated for cuboid cross section, e.g. in Mathematica:
    // 
    // Integrate[rho[x]*y^0, {z, -H/2, H/2}, {y, -K/2, K/2}]
    return _H * _K * rho ( i );
}



double BeamGeometryCuboid::b1 ( const int i ) const {
    // b0 integral evaluated for cuboid cross section, e.g. in Mathematica:
    // 
    // Integrate[rho[x]*y^1, {z, -H/2, H/2}, {y, -K/2, K/2}]
    return 0.0;
}


struct B0mfunc {
    B0mfunc ( const BeamGeometryCuboid &geom ) : _geom ( geom ) {};

    double operator() ( const int i ) const {
        return _geom.b0 ( i );
    };
private:
    const BeamGeometryCuboid &_geom;
};



double BeamGeometryCuboid::B0m ( const int i ) const {
    B0mfunc func ( *this );

    const double h = get_h();

    // integrate over the length of the beam from 0 to i
    double val= TrapMethod::trapezMethod< B0mfunc> ( func, i+1, h );

    return val;
}




double BeamGeometryCuboid::B0_fnc ( const int i ) const {
    return B0m ( getM()-1 ) -B0m ( i );
}



double BeamGeometryCuboid::B0 ( const int i ) const {
    // the values of B0 have been calculated in the constructor, so we just return it here
    return _B0vec[i];
}




double BeamGeometryCuboid::c2 ( const int i ) const {
    // c2 integral evaluated for cuboid cross section, e.g. in Mathematica:
    // 
    // Integrate[(\[Kappa][x]/8)*y^2, {z, -H/2, H/2}, {y, -K/2, K/2}]
    return ( 1.0/96.0 ) * _H * pow ( _K, 3.0 ) * kappa ( i );
}



double BeamGeometryCuboid::c3 ( const int i ) const {
    // c3 integral evaluated for cuboid cross section, e.g. in Mathematica:
    // 
    // Integrate[(\[Kappa][x]/8)*y^3, {z, -H/2, H/2}, {y, -K/2, K/2}]
    return 0.0;
}



double BeamGeometryCuboid::c4 ( const int i ) const {
    // c4 integral evaluated for cuboid cross section, e.g. in Mathematica:
    // 
    // Integrate[(\[Kappa][x]/8)*y^4, {z, -H/2, H/2}, {y, -K/2, K/2}]    
    return ( 1.0/640.0 ) * _H * pow ( _K, 5.0 ) * kappa ( i );
}
