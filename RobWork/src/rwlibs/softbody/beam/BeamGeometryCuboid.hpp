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

#ifndef CUBOIDGEOMETRY_HPP
#define CUBOIDGEOMETRY_HPP

#include <ostream>
#include <sstream>

#include "BeamGeometry.hpp"

namespace rwlibs {
namespace softbody {

class BeamGeometryCuboid : public BeamGeometry
{
    public:
    BeamGeometryCuboid(double dx, 
			       double dy, 
			       double dz, 
			       const std::vector<double> &Exvec,
		   const std::vector<double> &vxvec,
			       const std::vector<double> &rhovec
			       );
    
    public:
	virtual double b0(const int i) const;
	virtual double b1(const int i) const;
	
	virtual double c2(const int i) const;
	virtual double c3(const int i) const;
	virtual double c4(const int i) const;
	
	virtual double B0(const int i) const;
    private:

	double B0_fnc(const int i) const;
	double B0m(const int i) const;
	
    public:
	double getH(void) const { return _H;};
	double getK(void) const { return _K;};
	
	friend std::ostream& operator<<(std::ostream& out, const BeamGeometryCuboid& obj) {
	    std::stringstream str;
	    
	    const BeamGeometry &base(obj);
	    // embed base class?
	        
	    str << "BeamGeometryCuboid {" << base << "  this: " << &obj << ", H:" << obj.getH() << ", K:" << obj.getK() << "}";
	    
	    return out << str.str();
	};
	
	
    private:
	const double _H, _K;
	std::vector<double> _B0vec;
};
}}

#endif // CUBOIDGEOMETRY_HPP
