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


#ifndef EBBEAM_HPP
#define EBBEAM_HPP

namespace rwlibs {
namespace softbody {

class EBBeam {
	public:
		EBBeam(
		const double H, 
		   const double K, 
		   const double L, 
		   const double E, 
		   const double rho,
		   const double h,
         const double g2
		   );
		
		double operator() (const int i) const;
		
		double d(const int i) const;
    
    private:   
		const double _H, _K, _L, _E, _rho, _h;
		
		double _J;
		double _q;
};
}};

#endif // EBBEAM_HPP
