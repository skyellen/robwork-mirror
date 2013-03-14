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


#include "BeamStartGuess.hpp"

#include <rw/math/Rotation2D.hpp>
#include <rw/math/Vector2D.hpp>

#include <math.h>

using namespace rwlibs::softbody;


BeamStartGuess::BeamStartGuess ( const double uxTCPy, const double len, const double yTCP, const double epsSafe, const rw::math::Rotation3D< double >& rot ) : _uxTCPy ( uxTCPy ), _len ( len ), _yTCP ( yTCP ), _epsSafe ( epsSafe ), _rot ( rot ) {}

double BeamStartGuess::curveY ( const double t ) {
    _epsSafe + _yTCP * exp ( - ( -_uxTCPy/_yTCP ) * t );
}



double BeamStartGuess::curveDY ( const double t ) {
    // E^((t uxTCPy)/yTCP) uxTCPy
    return exp ( ( ( t * _uxTCPy ) / _yTCP ) ) * _uxTCPy;
}



double BeamStartGuess::curveDX ( const double t ) {
    return sqrt ( 1.0 - exp ( ( 2 *t* _uxTCPy ) /_yTCP ) * pow ( _uxTCPy, 2.0 ) );
}



rw::math::Vector2D< double > BeamStartGuess::Dcurve ( const double t ) {
    rw::math::Vector2D<double> vec ( curveDX ( t ), curveDY ( t ) );
    
    return -vec;

    //return rw::math::inverse< double > ( _rot ) * vec;
}



boost::numeric::ublas::vector< double > BeamStartGuess::curve ( const int M ) {
    boost::numeric::ublas::vector<double> vec ( M );
    for ( int i = 1; i < M +1; i++ ) {
        // ArcSin[Dcurve[k Len/M][[2]]]
        rw::math::Vector2D<double> point = Dcurve ( i * ( _len / double ( M ) ) );
        vec[i-1] = asin ( point[1] );
    }

    return vec;
}



double BeamStartGuess::operator() ( const double gam ) {
    // gam = gam /. FindRoot[uxTCPy Len Exp[-gam Len] == -yTCP + epsSafe, {gam, 0}]}];
// 	std::cout << "blag: gam: " << gam << std::endl;

    double res = _uxTCPy * _len * exp ( -gam * _len ) 	+ _yTCP - _epsSafe;

// 	std::cout << "blag: res: " <<  res << std::endl;

    return res;
}

