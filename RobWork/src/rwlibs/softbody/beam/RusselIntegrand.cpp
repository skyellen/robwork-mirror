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


#include "RusselIntegrand.hpp"

#include <rw/common/macros.hpp>
#include <rwlibs/softbody/beam/BeamGeometry.hpp>

using namespace rwlibs::softbody;

RusselIntegrand::RusselIntegrand ( const BeamGeometry& geom, const boost::numeric::ublas::vector< double, boost::numeric::ublas::unbounded_array< double, std::allocator< double > > >& a, const boost::numeric::ublas::vector< double, boost::numeric::ublas::unbounded_array< double, std::allocator< double > > >& da ) :
    _geom ( geom ),
    _a ( a ),
    _da ( da ) {

}



double RusselIntegrand::eg ( const int i ) const {
    RW_ASSERT ( i < ( int ) _a.size() );
    const double &ax = _a[i];

    const double &g1 = _geom.g1();
    const double &g2 = _geom.g2();
    const double b0 = _geom.b0 ( i );
    const double b1 = _geom.b1 ( i );
    const double B0 = _geom.B0 ( i );
    const double x = i * _geom.get_h();

    const double Eg = ( g1*B0 + g2*b1 ) * cos ( ax ) + ( g2*B0 - g1*b1 ) * sin ( ax ) - g1*b0 * x - g2*b1;
    return Eg;
}



double RusselIntegrand::ee ( const int i ) const {
    RW_ASSERT ( i < ( int ) _da.size() );
    const double &dax = _da[i];

    const double c2 = _geom.c2 ( i );
    const double c3 = _geom.c3 ( i );
    const double c4 = _geom.c4 ( i );

    const double Ee = 4 * c2 * pow ( dax, 2.0 ) + 4 * c3 * pow ( dax, 3.0 ) + c4 * pow ( dax, 4.0 );
    return Ee;
}



double RusselIntegrand::operator() ( const int i ) const {
    return eg ( i ) + ee ( i );
}



RusselIntegrandEonly::RusselIntegrandEonly ( const BeamGeometry& geom, const boost::numeric::ublas::vector< double >& a, const boost::numeric::ublas::vector< double >& da ) :
    RusselIntegrand ( geom, a, da ) {
}



double RusselIntegrandEonly::operator() ( const int i ) const {
    return ee ( i );
}
