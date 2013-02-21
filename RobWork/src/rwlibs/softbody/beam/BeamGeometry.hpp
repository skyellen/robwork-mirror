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

namespace rwlibs {
namespace softbody {

class BeamGeometry
{
public:
    BeamGeometry(double L,
                 const std::vector<double> &Exvec,
                 const std::vector<double> &vxvec,
                 const std::vector<double> &rhovec
                );

public:
    // stuff implemented here
    double Ex(const int i) const;
    double vx(const int i) const;
    double rho(const int i) const;

    double kappa(const int i) const;

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

protected:
    double _L;

    int _NSlices;

    double _a, _b;
    double _h;
private:
    std::vector<double> _Exvec;
    std::vector<double> _vxvec;
    std::vector<double> _rhovec;


};
}
}

#endif // BEAMGEOMETRY_HPP
