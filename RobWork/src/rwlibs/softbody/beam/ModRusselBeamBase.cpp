/*
    Copyright 2013 <copyright holder> <email>

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


#include "ModRusselBeamBase.hpp"

using namespace rwlibs::softbody;

ModRusselBeamBase::ModRusselBeamBase ( 
boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
boost::shared_ptr< rwlibs::softbody::BeamObstaclePlane > obstaclePtr, 
int M ) 
:
_geomPtr ( geomPtr ), 
_obstaclePtr ( obstaclePtr ),
_M ( M )
{

}




ModRusselBeamBase::~ModRusselBeamBase() {

}



double ModRusselBeamBase::getAccuracy ( void ) const {
    return _accuracy;
}



void ModRusselBeamBase::setAccuracy ( double acc ) {
    _accuracy = acc;
}

boost::shared_ptr< BeamGeometry > ModRusselBeamBase::getGeometry ( void ) const {
    return _geomPtr;
}


boost::shared_ptr< BeamObstaclePlane > ModRusselBeamBase::getObstacle ( void ) const {
    return _obstaclePtr;
}

int ModRusselBeamBase::getM ( void ) const {
    return _M;
}



rw::math::Transform3D< double > ModRusselBeamBase::get_planeTbeam ( void ) const {
    const rw::math::Transform3D<> &Tbeam = getGeometry()->getTransform();
    rw::math::Transform3D<> planeTbeam = getObstacle()->compute_planeTbeam ( Tbeam );

    return planeTbeam;
}



double ModRusselBeamBase::get_thetaTCP ( void ) const {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    double thetaTCP = getObstacle()->get_thetaTCP ( planeTbeam );

    return thetaTCP;
}



double ModRusselBeamBase::get_yTCP ( void ) const {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    double yTCP = getObstacle()->get_yTCP ( planeTbeam );

    return yTCP;
}



double ModRusselBeamBase::get_uxTCPy ( void ) const {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    return planeTbeam.R() ( 1, 0 );
}



double ModRusselBeamBase::get_uyTCPy ( void ) const {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    return planeTbeam.R() ( 1, 1 );
}

double ModRusselBeamBase::get_h ( void ) const {
    // TODO calculate once and set var
    double a = getGeometry()->get_a();
    double b = getGeometry()->get_b();

    return ( b-a ) / ( getM()-1 );
}





