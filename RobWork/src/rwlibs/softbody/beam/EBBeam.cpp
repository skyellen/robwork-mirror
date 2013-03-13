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


#include "EBBeam.hpp"

#include <math.h>

using namespace rwlibs::softbody;

EBBeam::EBBeam(const double H, const double K, const double L, const double E, const double rho, const double h, const double g2)
:
			_H(H),
			_K(K),
			_L(L),
			_E(E),
			_rho(rho),
			_h(h)
		   {
			_J = (_H * pow(_K, 3.0) ) / 12.0;
			_q = g2 * _rho * _K * _H;
		};


		
		
double EBBeam::operator()(const int i) const
 {
		const double x = i * _h;
		
		
		
		return (_q * pow(x, 2.0) * (6.0 * pow(_L, 2.0) - 4.0 * _L * x + pow(x, 2.0))) / (24.0 * _E * _J);
		
		};
		
		
		
double EBBeam::d(const int i) const
 {
		const double x = i * _h;
		
		return (_q * x * (3 * pow(_L, 2.0) - 3 *  _L * x + pow(x, 2.0)))/(6 *  _E * _J);
		
		};
		
