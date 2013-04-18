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

#include "BeamGeometry.hpp"

#include <assert.h>

#include <math.h>
#include <iostream>



//#include "Interpolation.hpp"

using namespace std;
using namespace rw::math;
using namespace rwlibs::softbody;

BeamGeometry::BeamGeometry(
		double L, 
		const std::vector<double> &Exvec,
		   const std::vector<double> &vxvec,
		const std::vector<double> &rhovec,
        const rw::math::Transform3D<> &wTb,
                 const rw::math::Vector3D<> &G
		)  
		: _L(L), _a(0.0), _b(L), _Exvec(Exvec), _vxvec(vxvec), _rhovec(rhovec), _wTb(wTb), _G(G)
{
	assert (Exvec.size() == rhovec.size() );
	assert (vxvec.size() == Exvec.size() );
	_M = _Exvec.size();

	const int Nintervals = _M -1;

	_h = (_b - _a) / Nintervals;
}




BeamGeometry::~BeamGeometry() {

}





void BeamGeometry::setTransform ( const rw::math::Transform3D< double >& T ) {
    _wTb = T;
}


rw::math::Transform3D< double > BeamGeometry::getTransform ( void ) const {
    return _wTb;
}



rw::math::Vector3D< double > BeamGeometry::getG ( void ) const {
    return _G;
}


void BeamGeometry::setG ( const rw::math::Vector3D< double >& G ) {
    _G = G;
}




double BeamGeometry::g1 ( void ) const  {
    //const double uxTCPy = getTransform().R() ( 1,0 );
    // -G {uxTCPy, uyTCPy}
    
    Rotation3D<double > beamRTranspose =  getTransform().R().inverse();
    Vector3D<> g =beamRTranspose * getG();
    
    // TODO hack with signs - due to post-multiplication?
    return g[0];

}


double BeamGeometry::g2 ( void ) const  {
    //const double uyTCPy = _T _rot ( 1,1 );
    //_g2 = _G * _uyTCPy;
    Rotation3D<double > beamRTranspose = getTransform().R().inverse();
    Vector3D<> g =beamRTranspose * getG();
    
    return g[1];
}

/*
double BeamGeometry::get_uxTCPy() const { 
    return getTransform().R() (1, 0);
}


double BeamGeometry::get_uyTCPy() const {
    return getTransform().R() (1, 1);
}*/



 double BeamGeometry::Ex(const int i) const
{
    /*
	double y = Interpolation::interpolateVector< std::vector<double> >(_Exvec, xi, _a, _b, _h);
	return y;
	*/
	
	//int i = (xi - _a) / _h;
	
	return _Exvec[i];
}


 double BeamGeometry::vx(const int i) const
{
    
//     double y =  Interpolation::interpolateVector< std::vector<double> >(_vxvec, xi, _a, _b, _h);
//     return y;
    
    
    
    //int i = (xi - _a) / _h;
    
    return _vxvec[i];
    
}



 double BeamGeometry::rho(const int i) const
{
//     double y =  Interpolation::interpolateVector< std::vector<double> >(_rhovec, xi, _a, _b, _h);
// 	return y;
	
	//int i = (xi - _a) / _h;
	
	return _rhovec[i];
}


 double BeamGeometry::kappa(const int i) const
{
    return Ex(i) / (1.0 - pow(vx(i), 2.0) );
}
