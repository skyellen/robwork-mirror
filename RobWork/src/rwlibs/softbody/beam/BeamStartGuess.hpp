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
        /** @addtogroup softbody */
/*@{*/
    /**
     * @brief Utility functions for setting starting guesses for a beam model
     * 
     * @note These functions only set the angle of the deformation (contained in avec). Be sure to integrate this yourself should you need the deformation vector!.
     **/
    class BeamStartGuess {
public:
    /**
     * @brief sets a starting guess with all elements of avec initialized to zero
     *
     * @param avec reference to starting guess vector to be set 
     * @param beamPtr pointer to the beam
     **/
    static void setZeroStartingGuess( boost::numeric::ublas::vector<double> &avec, boost::shared_ptr< ModRusselBeamBase > beamPtr );
    
    
    
    /**
     * @brief sets a starting guess for a cuboid beam initialized to that of an analytical Euler-Bernoulli beam model
     *
     * @param avec reference to starting guess vector to be set 
     * @param beamPtr pointer to the beam geometry
     **/
    static void setEulerStartingGuess ( boost::numeric::ublas::vector<double> &avec, boost::shared_ptr< rwlibs::softbody::BeamGeometryCuboid > beamGeomPtr );
};
/*@}*/
}}


#endif // BEAMSTARTGUESS_HPP
