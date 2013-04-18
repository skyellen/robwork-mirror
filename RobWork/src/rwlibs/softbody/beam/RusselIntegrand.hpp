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


#ifndef RUSSELINTEGRAND_HPP
#define RUSSELINTEGRAND_HPP

#include <boost/numeric/ublas/vector.hpp>

#include <rw/common/macros.hpp>

#include <rwlibs/softbody/beam/BeamGeometry.hpp>

namespace rwlibs {
namespace softbody {

struct RusselIntegrand {
    RusselIntegrand (
        const BeamGeometry &geom,
        const boost::numeric::ublas::vector<double>& a,
        const boost::numeric::ublas::vector<double>& da
    ) ;;

    // gravitational energy per unit volume
    double eg ( const int i ) const ;;

    // elastic energy per unit volume
    double ee ( const int i ) const ;;

    // total energy per unit volume
    double operator() ( const int i ) const ;;

private:
    const BeamGeometry &_geom;
    const boost::numeric::ublas::vector<double>& _a;
    const boost::numeric::ublas::vector<double>& _da;
};





struct RusselIntegrandEonly : public RusselIntegrand {
    RusselIntegrandEonly (
        const BeamGeometry &geom,
        const boost::numeric::ublas::vector<double>& a,
        const boost::numeric::ublas::vector<double>& da
    ) ;;

    // only elastic energy
    double operator() ( const int i ) const ;;
};

}}


#endif // RUSSELINTEGRAND_HPP
