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

#include "CuboidGeometry.hpp"

#include <math.h>
#include <iostream>
#include <assert.h>

#include "TrapMethod.hpp"




using namespace std;


CuboidGeometry::CuboidGeometry(double dx, double dy, double dz, const std::vector< double >& Exvec, const std::vector< double >& vxvec, const std::vector< double >& rhovec)
: 
Geometry(dx, Exvec, vxvec, rhovec), 
_K(dy),
_H(dz),
_B0vec(_NSlices)
{
//     const int M = _NSlices;
//     const double h = _h;
    
    for (int i = 0; i < _NSlices; i++) {
	//const double xi = i * h;
	
	if (0 == i)
	    _B0vec[i] = B0m(_NSlices-1);    
	else
	    _B0vec[i] = B0_fnc(i);
	
 	std::cout << "_B0vec[i]: " << _B0vec[i] << std::endl;
    }
//     assert(false);
};




double CuboidGeometry::b0(const int i) const
{
    return _H * _K * rho(i);
}



double CuboidGeometry::b1(const int i) const
{
    return 0.0;
}


struct B0mfunc {
    B0mfunc(const CuboidGeometry &geom) : _geom(geom) {}; // must set function somehoW? FEELS dirty
    
    double operator() (const int i) const {
	return _geom.b0(i);
    };
	private:
	    const CuboidGeometry &_geom;
    };



double CuboidGeometry::B0m(const int i) const
{

    
    B0mfunc func(*this);
    const int M = _NSlices;
    const double h = _h;
    
    //double i = (xi - 0) / h;
    
    // find two closest integer slices
    //int i0 = round(i);
//     std::cout << "i: " << i << std::endl;
//     std::cout << "i0: " << i0 << std::endl;
    //int i1 = i0 + 1;
    
//     assert(i0 != 0);
    
    double val;
//     if (0 == i0)
// 	val = 0.0;
//     else {
// 	assert(i0 >= 0);
    
	val = TrapMethod::trapezMethod< B0mfunc>(func, i+1, h);

//     }
    
     
//     std::cout << "val: " << val << std::endl;
    
    return val;
}




double CuboidGeometry::B0_fnc(const int i) const
{
    return B0m(_NSlices-1) -B0m(i);
}



double CuboidGeometry::B0(const int i) const
{
//     std::cout << "(xi - 0) / _h: " << (xi - 0) / _h << std::endl;
    //int i = (xi - 0) / _h;
//     std::cout << "i: " << i << std::endl;
    
    return _B0vec[i];
}




double CuboidGeometry::c2(const int i) const
{
    return (1.0/96.0) * _H * pow(_K, 3.0) * kappa(i);
}

double CuboidGeometry::c3(const int i) const
{
    return 0.0;
}


double CuboidGeometry::c4(const int i) const
{
   return  (1.0/640.0) * _H * pow(_K, 5.0) * kappa(i);
}

