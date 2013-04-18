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

#ifndef BEAMGEOMETRY_HPP
#define BEAMGEOMETRY_HPP

#include <vector>
#include <ostream>
#include <sstream>

#include <rw/math/Transform3D.hpp>

namespace rwlibs {
namespace softbody {

class BeamGeometry
{
public:
    BeamGeometry(double L,
                 const std::vector<double> &Exvec,
                 const std::vector<double> &vxvec,
                 const std::vector<double> &rhovec,
                 const rw::math::Transform3D<> &wTb,
                 const rw::math::Vector3D<> &G
                );
    
    virtual ~BeamGeometry();
    
public:
    void setTransform(const rw::math::Transform3D<> &T);
    rw::math::Transform3D<> getTransform(void) const;
    
    void setG(const rw::math::Vector3D<> &G);
    rw::math::Vector3D<> getG(void) const;

public:
    // stuff implemented here
    double Ex(const int i) const;
    double vx(const int i) const;
    double rho(const int i) const;

    double kappa(const int i) const;
    
    double g1 ( void ) const;
    double g2 ( void ) const;
    
        
//     double get_uxTCPy() const;
//     double get_uyTCPy() const;
    
//     double get_yTCP() const;
//     double get_thetaTCP() const;

public:
    // stuff implemented by derived classes,  specific to ModRussel beam
    // the idea is that the derived classes may perform the integration as they'd like
    virtual double b0(const int i) const = 0;
    virtual double b1(const int i) const = 0;

    virtual double c2(const int i) const = 0;
    virtual double c3(const int i) const = 0;
    virtual double c4(const int i) const = 0;

    virtual double B0(const int i) const = 0;

public:
    double get_a(void) const {
        return _a;
    };
    double get_b(void) const {
        return _b;
    };
    double get_h(void) const {
        return _h;
    };

    double getL(void) const {
        return _L;
    };
    
    int getM(void) const {
        return _M;
    };
    
public:
	friend std::ostream& operator<<(std::ostream& out, const BeamGeometry& obj) {
	    std::stringstream str;
	        
	    str << "BeamGeometry {a: " << obj.get_a() << ", b: " << obj.get_b() << ", h: " << obj.get_h() << ", L: " << obj.getL() << "}";
	    
	    return out << str.str();
	};

private:
    double _L;

    int _M;

    double _a, _b;
    double _h;
private:
    std::vector<double> _Exvec;
    std::vector<double> _vxvec;
    std::vector<double> _rhovec;

    rw::math::Transform3D<> _wTb; // world to beam transform
    rw::math::Vector3D<> _G; 
};
}
}

#endif // BEAMGEOMETRY_HPP
