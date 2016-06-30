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


#ifndef RWLIBS_SOFTBODY_RUSSELINTEGRAND_HPP
#define RWLIBS_SOFTBODY_RUSSELINTEGRAND_HPP

#include <boost/numeric/ublas/vector.hpp>

namespace rwlibs {
namespace softbody {
class BeamGeometry;

/** @addtogroup softbody */
/*@{*/

/**
 * @brief Implementation of the Modified Russel energy density functions
 **/
struct RusselIntegrand {
    /**
     * @brief Constructor
     *
     * @param geom the geometry of the beam
     * @param a vector containing the angle of the deformation
     * @param da vector containing the deriatives of the angle of deformation
     **/
    RusselIntegrand (
        const BeamGeometry &geom,
        const boost::numeric::ublas::vector<double>& a,
        const boost::numeric::ublas::vector<double>& da
    ) ;;


    /**
     * @brief returns the gravitional energy per cross section
     *
     * @param i index of cross section
     * @return gravitional energy
     **/
    double eg ( const int i ) const ;;

    /**
     * @brief returns the elastic energy per cross section
     *
     * @param i index of cross section
     * @return elastic energy
     **/
    double ee ( const int i ) const ;;

    
    /**
     * @brief returns the total elastic energy per cross section eg+ee
     *
     * @param i index of cross section
     * @return total elastic energy per cross section
     **/
    double operator() ( const int i ) const ;;

private:
    const BeamGeometry &_geom;
    const boost::numeric::ublas::vector<double>& _a;
    const boost::numeric::ublas::vector<double>& _da;
};





/**
 * @brief Implementation of the Modified Russel elastic energy density function
 * 
 **/
struct RusselIntegrandEonly : public RusselIntegrand {
    /**
     * @brief Constructor
     *
     * @param geom the geometry of the beam
     * @param a vector containing the angle of the deformation
     * @param da vector containing the deriatives of the angle of deformation
     **/
    RusselIntegrandEonly (
        const BeamGeometry &geom,
        const boost::numeric::ublas::vector<double>& a,
        const boost::numeric::ublas::vector<double>& da
    ) ;;

    /**
     * @brief returns the elastic energy per cross section
     *
     * @param i index of cross section
     * @return elastic energy
     **/
    double operator() ( const int i ) const ;;
};
/*@}*/
}}


#endif // RUSSELINTEGRAND_HPP
