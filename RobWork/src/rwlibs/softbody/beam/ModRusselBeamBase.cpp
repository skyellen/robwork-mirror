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


#include "ModRusselBeamBase.hpp"

using namespace rwlibs::softbody;

ModRusselBeamBase::ModRusselBeamBase ( 
boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
boost::shared_ptr< rwlibs::softbody::BeamObstaclePlane > obstaclePtr, 
int M ) 
:
_geomPtr ( geomPtr ), 
_obstaclePtr ( obstaclePtr ),
_M ( M ),
_useNoUpwardConstraint ( false ), 
_nIntegralConstraints ( 1 ),
_useHingeConstraint( false )
{

}




ModRusselBeamBase::~ModRusselBeamBase() {

}



// integrates x-component of angle, assuming implictly a(0) = 0;
void ModRusselBeamBase::integrateAngleU ( boost::numeric::ublas::vector< double >& U, const boost::numeric::ublas::vector< double >& avec ) {
    const double h = ( getGeometry()->get_b() - getGeometry()->get_a() ) / avec.size();

    U[0] = 0.0;
    for ( int end = 0; end < ( int )  avec.size(); end++ ) {
        const double f0 = cos ( 0.0 );
        const double fL = cos ( avec[end] );

        double sum = 0.0;
        for ( int i = 0; i < ( int ) end; i++ )
            sum += cos ( avec[i] );

        U[end+1] = ( h / 2.0 ) * ( f0 + fL ) + h * sum;
    }
}

// integrates y-component of angle, assuming implictly a(0) = 0;
void ModRusselBeamBase::integrateAngleV ( boost::numeric::ublas::vector< double >& V, const boost::numeric::ublas::vector< double >& avec ) {
    const double h = ( getGeometry()->get_b() - getGeometry()->get_a() ) / avec.size();
//     std::cout << "h: " << h << std::endl;
//     std::cout << "avec.size(): " << avec.size() << std::endl;

    V[0] = 0.0;
    for ( int end = 0; end < ( int ) avec.size(); end++ ) {
        const double f0 = sin ( 0.0 );
        const double fL = sin ( avec[end] );

        double sum = 0.0;
        for ( int i = 0; i < ( int ) end; i++ )
            sum += sin ( avec[i] );

        V[end+1] = ( h / 2.0 ) * ( f0 + fL ) + h * sum;
    }
}


void ModRusselBeamBase::computeIntegralIndicies ( void ) {
    _integralConstraintIdxList.clear();
    int N = getM() -1;
    
    if ( get_nIntegralConstraints() == 1) {
        int idx = N - 1;
        _integralConstraintIdxList.push_back ( idx );
    }   
    else if ( get_nIntegralConstraints() > 0 ) {
        const double hi = N / get_nIntegralConstraints();
               
        if (hi < 2) {
            RW_THROW("Number of integral constraints must be less than M/2");
        }
        else {        
            for ( int i = 1; i < get_nIntegralConstraints() + 1; i++ ) {
                int idx = ( int ) ceil ( double ( i ) * hi ) ;
                _integralConstraintIdxList.push_back ( idx );
            }
        }
    }
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



double ModRusselBeamBase::get_uxTCPy ( const rw::math::Transform3D< double > planeTbeam )  {
    return planeTbeam.R() ( 1, 0 );
}


double ModRusselBeamBase::get_uxTCPy ( void ) const {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    
    return get_uxTCPy(planeTbeam);
}



double ModRusselBeamBase::get_uyTCPy ( const rw::math::Transform3D< double > planeTbeam )  {
    return planeTbeam.R() ( 1, 1 );
}


double ModRusselBeamBase::get_uyTCPy ( void ) const {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    return get_uyTCPy(planeTbeam);
}

double ModRusselBeamBase::get_h ( void ) const {
    // TODO calculate once and set var
    double a = getGeometry()->get_a();
    double b = getGeometry()->get_b();

    return ( b-a ) / ( getM()-1 );
}



/*
void ModRusselBeamBase::setUseNoUpwardConstraint ( bool val ) {
    _useNoUpwardConstraint = val;
}


void ModRusselBeamBase::setUseHingeConstraint ( bool val ) {
    _useHingeConstraint = val;
}
*/

/*

void ModRusselBeamBase::setMuDecrementFactor ( double decFactor ) {
    _muDec = decFactor;
}


void ModRusselBeamBase::setMuStart ( double muStart ) {
    _muStart = muStart;
}
*/


void ModRusselBeamBase::set_nIntegralConstraints ( int nIntegralConstraints ) {
    _nIntegralConstraints = nIntegralConstraints;
}

int ModRusselBeamBase::get_nIntegralConstraints ( void ) const {
    return _nIntegralConstraints;
}




