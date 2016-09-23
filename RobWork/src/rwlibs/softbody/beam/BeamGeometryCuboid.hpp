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

#ifndef RWLIBS_SOFTBODY_CUBOIDGEOMETRY_HPP
#define RWLIBS_SOFTBODY_CUBOIDGEOMETRY_HPP

#include <ostream>
#include <sstream>

#include "BeamGeometry.hpp"

namespace rwlibs {
namespace softbody {
/** @addtogroup softbody */
/*@{*/

/**
* @brief A cuboid beam
*
* Implements the BeamGeometry class by the means of analytical integration of a cuboid geometry
*/
class BeamGeometryCuboid : public BeamGeometry {
public:

    /**
    * @brief Constructor for cuboid deformation geometry
    *
    * @param dx length in x (L)
    * @param dy length in y
    * @param dz length in z
    * @param Exvec vector of Young's modulus for each cross beam cros section, in MPa
    * @param vxvcec vector of Poisson's ratio for each cross beam cros section
    * @param rhovec vector of mass density for each cross beam cros section, in kg/mm^3
    * @param wTb world to base of beam transform
    * @param G vector of gravity, m/s^2
    */
    BeamGeometryCuboid (
        double dx,
        double dy,
        double dz,
        const std::vector<double> &Exvec,
        const std::vector<double> &vxvec,
        const std::vector<double> &rhovec,
        const rw::math::Transform3D<> &wTb,
        const rw::math::Vector3D<> &G
    );

    virtual ~BeamGeometryCuboid();

public:
    virtual double b0 ( const int i ) const;
    virtual double b1 ( const int i ) const;

    virtual double c2 ( const int i ) const;
    virtual double c3 ( const int i ) const;
    virtual double c4 ( const int i ) const;

    virtual double B0 ( const int i ) const;

public:
    /**
    * @brief Returns the thickness of the beam in the z-direction
    *
    * @return thickness in z
    */
    double getH ( void ) const {
        return _H;
    };



    /**
    * @brief Returns the thickness of the beam in the y-direction
    *
    * @return thickness in y
    */
    double getK ( void ) const {
        return _K;
    };



    /**
    * @brief Outputs cuboid beam to stream
    *
    * @param out
    * @param obj
    *
    * @return
    */
    friend std::ostream& operator<< ( std::ostream& out, const BeamGeometryCuboid& obj ) {
        std::stringstream str;

        const BeamGeometry &base ( obj );

        str << "BeamGeometryCuboid {" << base << "  this: " << &obj << ", H:" << obj.getH() << ", K:" << obj.getK() << "}";

        return out << str.str();
    };



private:
    double B0_fnc ( const int i ) const;
    double B0m ( const int i ) const;


private:
    const double _H; // thickness in z
    const double _K; // thickness in y
    std::vector<double> _B0vec; // vector holding the cached values of B0
};
/*@}*/
}
}

#endif // CUBOIDGEOMETRY_HPP
