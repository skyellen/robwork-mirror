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

#include "BeamGeometryCuboid.hpp"

#include <math.h>
#include <iostream>
#include <assert.h>

//#include "../numerics/TrapMethod.hpp"
#include "rwlibs/softbody/numerics/TrapMethod.hpp"




using namespace std;
using namespace rwlibs::softbody;


BeamGeometryCuboid::BeamGeometryCuboid(
    double dx,
    double dy, 
    double dz, 
                 const std::vector<double> &Exvec,
                 const std::vector<double> &vxvec,
                 const std::vector<double> &rhovec,
                 const rw::math::Transform3D<> &wTb,
                 const rw::math::Vector3D<> &G    
)
: 
BeamGeometry(dx, Exvec, vxvec, rhovec, wTb, G), 
_H(dz),
_K(dy)
{
//     const int M = _NSlices;
//     const double h = _h;
    _B0vec.resize(getM());
    
    for (int i = 0; i < getM(); i++) {
	//const double xi = i * h;
	
	if (0 == i)
	    _B0vec[i] = B0m(getM()-1);    
	else
	    _B0vec[i] = B0_fnc(i);
	
//  	std::cout << "_B0vec[i]: " << _B0vec[i] << std::endl;
    }
//     assert(false);
};



BeamGeometryCuboid::~BeamGeometryCuboid() {

}





double BeamGeometryCuboid::b0(const int i) const
{
    return _H * _K * rho(i);
}



double BeamGeometryCuboid::b1(const int i) const
{
    return 0.0;
}


struct B0mfunc {
    B0mfunc(const BeamGeometryCuboid &geom) : _geom(geom) {}; // must set function somehoW? FEELS dirty
    
    double operator() (const int i) const {
	return _geom.b0(i);
    };
	private:
	    const BeamGeometryCuboid &_geom;
    };



double BeamGeometryCuboid::B0m(const int i) const
{

    
    B0mfunc func(*this);
    //const int M = _NSlices;
    const double h = get_h();
    
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




double BeamGeometryCuboid::B0_fnc(const int i) const
{
    return B0m(getM()-1) -B0m(i);
}



double BeamGeometryCuboid::B0(const int i) const
{
//     std::cout << "(xi - 0) / _h: " << (xi - 0) / _h << std::endl;
    //int i = (xi - 0) / _h;
//     std::cout << "i: " << i << std::endl;
    
    return _B0vec[i];
}




double BeamGeometryCuboid::c2(const int i) const
{
    return (1.0/96.0) * _H * pow(_K, 3.0) * kappa(i);
}

double BeamGeometryCuboid::c3(const int i) const
{
    return 0.0;
}


double BeamGeometryCuboid::c4(const int i) const
{
   return  (1.0/640.0) * _H * pow(_K, 5.0) * kappa(i);
}


// std::ostream& BeamGeometryCuboid::operator<<(std::ostream& out, const BeamGeometryCuboid& obj)
