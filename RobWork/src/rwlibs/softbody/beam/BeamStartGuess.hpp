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

#include <boost/shared_ptr.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector2D.hpp>

#include "BeamGeometryCuboid.hpp"
#include "ModRusselBeamBase.hpp"



namespace rwlibs {
namespace softbody {

class BeamStartGuess {
public:
    //BeamStartGuess(boost::shared_ptr< ModRusselBeamBase > beamPtr);
    
public:
    static void setZeroStartingGuess( boost::numeric::ublas::vector<double> &avec, boost::shared_ptr< ModRusselBeamBase > beamPtr );
    static void setEulerStartingGuess ( boost::numeric::ublas::vector<double> &avec, boost::shared_ptr< rwlibs::softbody::BeamGeometryCuboid > beamGeomPtr );

private:
//     boost::shared_ptr< ModRusselBeamBase > _beamPtr;
};
}}
/*
 *
    StartGuessFunc func(uxTCPy, LEN, yTCP, EPSSAFE, optm.getRot());
 *
 *
 */

#endif // BEAMSTARTGUESS_HPP
