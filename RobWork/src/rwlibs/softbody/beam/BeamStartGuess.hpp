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


#ifndef BEAMSTARTGUESS_HPP
#define BEAMSTARTGUESS_HPP

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector2D.hpp>

namespace rwlibs {
namespace softbody {

struct BeamStartGuess {
    BeamStartGuess ( const double uxTCPy,
                     const double len,
                     const double yTCP,
                     const double epsSafe,
                     const rw::math::Rotation3D<double> &rot
                   ) ;

    double operator() ( const double gam ) ;

    double curveY ( const double t ) ;

    double curveDY ( const double t ) ;

    double curveDX ( const double t ) ;

    rw::math::Vector2D<double> Dcurve ( const double t ) ;

    boost::numeric::ublas::vector<double> curve ( const int M ) ;

private:
    double _uxTCPy, _len, _yTCP, _epsSafe;
    const rw::math::Rotation3D<double> &_rot;
};
}}
/*
 *
    StartGuessFunc func(uxTCPy, LEN, yTCP, EPSSAFE, optm.getRot());
 *
 *
 */

#endif // BEAMSTARTGUESS_HPP
