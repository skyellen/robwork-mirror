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

#include "Geometry.hpp"

#include <assert.h>

#include <math.h>
#include <iostream>

#include "Interpolation.hpp"

using namespace std;
//using namespace Interpolation;

Geometry::Geometry(
		double L, 
		const std::vector<double> &Exvec,
		   const std::vector<double> &vxvec,
		const std::vector<double> &rhovec
		)  
		: _L(L), _a(0.0), _b(L), _Exvec(Exvec), _vxvec(vxvec), _rhovec(rhovec)
{
	assert (Exvec.size() == rhovec.size() );
	assert (vxvec.size() == Exvec.size() );
	_NSlices = _Exvec.size();

	const int Nintervals = _NSlices -1;

	_h = (_b - _a) / Nintervals;
}


 double Geometry::Ex(const int i) const
{
    /*
	double y = Interpolation::interpolateVector< std::vector<double> >(_Exvec, xi, _a, _b, _h);
	return y;
	*/
	
	//int i = (xi - _a) / _h;
	
	return _Exvec[i];
}


 double Geometry::vx(const int i) const
{
    
//     double y =  Interpolation::interpolateVector< std::vector<double> >(_vxvec, xi, _a, _b, _h);
//     return y;
    
    
    
    //int i = (xi - _a) / _h;
    
    return _vxvec[i];
    
}



 double Geometry::rho(const int i) const
{
//     double y =  Interpolation::interpolateVector< std::vector<double> >(_rhovec, xi, _a, _b, _h);
// 	return y;
	
	//int i = (xi - _a) / _h;
	
	return _rhovec[i];
}


 double Geometry::kappa(const int i) const
{
    return Ex(i) / (1.0 - pow(vx(i), 2.0) );
}
