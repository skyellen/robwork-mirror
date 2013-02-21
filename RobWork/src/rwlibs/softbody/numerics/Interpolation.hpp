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

#ifndef INTERPOLATION_HPP
#define INTERPOLATION_HPP

// #include <boost/numeric/ublas/vector.hpp>
// #include <vector>


#include <math.h>
#include <assert.h>
#include <iostream>

class Interpolation {
    public:
    
    template <class T>
    static double interpolateVector(const T &vec,  const double xi, const double a, const double b __attribute__((unused)), const double h) {
	//std::cout << "xi: " << xi << std::endl;
	
	//eassert(xi >= a);
	//eassert(xi <= b);
	
	// given xi, find closest i
	double i = (xi - a) / h;
	

	// find two closest integer slices
	int i0 = floor(i);
	int i1 = i0 + 1;
	
	//std::cout << "i0: " << i0 << std::endl;
	//std::cout << "i1: " << i1 << std::endl;
	
	assert(i0 >= 0);
	assert(i1 >= 0);
	
	double x0 = a + i0 * h;
	double x1 = a + i1 * h;
	
	//std::cout << "x0: " << x0 << std::endl;
	//std::cout << "x1: " << x1 << std::endl;
	
	assert(i0 <= (int)vec.size() -1);
	
	if (xi == x0) // requested value matches excactly
	    return vec[i0];
	
	
	assert(i1 <= (int)vec.size() -1);
	
	double y0 = vec[i0];
	double y1 = vec[i1];
	
	//std::cout << "y0: " << y0 << std::endl;
	//std::cout << "y1: " << y1 << std::endl;
	
	double y = linearInterpolation(xi, x0, x1, y0, y1);
	
	//std::cout << "y: " << y << std::endl;
	
	return y;    
    };
    
    private:
	static double linearInterpolation(double x, double x0, double x1, double y0, double y1)  {
	    double y = y0 + (x-x0) * ( (y1-y0) / (x1-x0) );
	    
	    return y;    
	}
	
};

#endif // INTERPOLATION_HPP
