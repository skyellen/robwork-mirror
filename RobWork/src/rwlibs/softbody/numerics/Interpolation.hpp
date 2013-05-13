/*
    Copyright [yyyy] [name of copyright owner]

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

#ifndef RWLIBS_SOFTBODY_INTERPOLATION_HPP
#define RWLIBS_SOFTBODY_INTERPOLATION_HPP

// #include <boost/numeric/ublas/vector.hpp>
// #include <vector>


#include <math.h>
#include <assert.h>
#include <iostream>

namespace rwlibs {
namespace softbody {
  /** @addtogroup softbody */
/*@{*/
/**
 * @brief Various routines related to interpolation
 **/
class Interpolation {
    public:
    
    template <class T>
    static double interpolateVector(const T &vec,  const double xi, const double a, const double b __attribute__((unused)), const double h) {

	// given xi, find closest i
	double i = (xi - a) / h;
	

	// find two closest integer slices
	int i0 = floor(i);
	int i1 = i0 + 1;
	
	assert(i0 >= 0);
	assert(i1 >= 0);
	
	double x0 = a + i0 * h;
	double x1 = a + i1 * h;
	
	assert(i0 <= (int)vec.size() -1);
	
	if (xi == x0) // requested value matches excactly
	    return vec[i0];
	
	
	assert(i1 <= (int)vec.size() -1);
	
	double y0 = vec[i0];
	double y1 = vec[i1];
	
	double y = linearInterpolation(xi, x0, x1, y0, y1);
	
	return y;    
    };
    
    private:
	static double linearInterpolation(double x, double x0, double x1, double y0, double y1)  {
	    double y = y0 + (x-x0) * ( (y1-y0) / (x1-x0) );
	    
	    return y;    
	}
	/*@}*/
};
}}

#endif // INTERPOLATION_HPP
