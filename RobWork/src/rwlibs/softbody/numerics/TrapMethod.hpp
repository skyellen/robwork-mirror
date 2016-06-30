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

#ifndef RWLIBS_SOFTBODY_TRAPMETHOD_HPP
#define RWLIBS_SOFTBODY_TRAPMETHOD_HPP

namespace rwlibs {
namespace softbody {
/** @addtogroup softbody */
/*@{*/
/**
 * @brief Implementation of the trapezoidal rule for integration
 **/
class TrapMethod {
public:
    /**
     * @brief routine for calculating a definite integral using the trapezoidal rule
     *
     * @param func the integrand as a function object
     * @param M the limit in the integration
     * @param h the stepsize
     * @return the evaluated integral
     **/
    template <class T>   
    static double inline trapezMethod ( T &func, const int M, const double h ) {
        const double f0 = func ( 0 );
        const double fL = func ( ( M-1 ) );

        double sum = 0.0;
        for ( int i = 1; i < M - 1; i++ )
            sum += func ( i );

        return ( h / 2.0 ) * ( f0 + fL ) + h * sum;
    }
};
/*@}*/
}
}

#endif // TRAPMETHOD_HPP
