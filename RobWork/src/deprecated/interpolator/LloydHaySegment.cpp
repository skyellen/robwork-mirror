/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "LloydHaySegment.hpp"

using namespace rw::interpolator;
using namespace rw::math;

LloydHaySegment::LloydHaySegment(const FunctionSegment& x1,
                                 const FunctionSegment& x2,
                                 double k,
                                 double tau):
    FunctionSegment(2*tau)
{
    _x1 = &x1;
    _x2 = &x2;
    _k = k;
    _tau = tau;
}

Q LloydHaySegment::getX(double t) const {
    double s = t/(getLength());
    double s3 = s*s*s;
    double s4 = s3*s;
    double s5 = s4*s;
    double alfa = 6*s5-15*s4+10*s3;
    double s6 = s5*s;
    double beta = s6-3*s5+3*s4-s3;
    // Get position of blended path segments
    Q x1 = _x1->getX(_x1->getLength()-_tau+t);
    Q x2 = _x2->getX(t-_tau);
    // Get velocity of the path segments
    Q v1 = _x1->getXd(_x1->getLength()-_tau+t);
    Q v2 = _x2->getXd(t-_tau);
    Q vd = v2-v1;
    return x1 + alfa* (x2-x1) - vd * _k * beta;
}

Q LloydHaySegment::getXd(double t) const {
    double s = t/(getLength());
    double s2 = s*s;
    double s3 = s2*s;
    double s4 = s3*s;
    double alfad = 30*s4-60*s3+30*s2;
    double s5 = s4*s;
    double betad = 6*s5-15*s4+12*s3-3*s2;

    double alfa = 6*s5-15*s4+10*s3;
    double s6 = s5*s;
    double beta = s6-3*s5+3*s4-s3;

    double index_x1 = _x1->getLength()-_tau+t;
    double index_x2 = t-_tau;

    // Get position of path segments
    Q x1 = _x1->getX(index_x1);
    Q x2 = _x2->getX(index_x2);
    // Get velocity of path segments
    Q v1 = _x1->getXd(index_x1);
    Q v2 = _x2->getXd(index_x2);
    // Get acceleration of path segments
    Q a1 = _x1->getXdd(index_x1);
    Q a2 = _x2->getXdd(index_x2);

    Q vd = v2-v1;
    return alfad*(x2-x1)-_k*(v2-v1)*betad-_k*beta*(a2-a1)+v1+alfa*(v2-v1);
}

Q LloydHaySegment::getXdd(double t) const {
    // TODO Why zero here?
    return Q(Q::ZeroBase(_x1->getX(0).size())); 
}
