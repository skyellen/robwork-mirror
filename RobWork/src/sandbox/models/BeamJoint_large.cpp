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


#include "BeamJoint.hpp"

// RW
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/State.hpp>

// NR
#include "nr3.h"
#include "stepper.h"
#include "stepperdopr853.h"
#include "odeint.h"
#include "roots.h"
#include "amoeba.h"

// Misc.
#include "util.hpp"

namespace rw { namespace models {

BeamJoint::BeamJoint(const std::string& name, const rw::math::Transform3D<>& transform) :
                                             Joint(name, 2), _transform(transform), _n(200), _L(1), _E(1), _I(1), _controlMode(false)
{
    _a.reserve(_n);
    _z.reserve(_n);
    _y.reserve(_n);
}

BeamJoint::~BeamJoint()
{
}

rw::math::Transform3D<> BeamJoint::getJointTransform(const rw::math::Q& q) const
{
    rw::math::Transform3D<> result;
    if(_controlMode) {
        if(shootingBack(q[1], q[0])) {
            result = getJointTransform(_F, _M, _L);
        } else {
            RW_WARN("Backward beam equation not solved!");
        }
    } else {
        result = getJointTransform(q[0], q[1], _L);
    }
    
    return result;
}

std::vector<double> BeamJoint::solveParameters(const rw::math::Q& q) const {
    std::vector<double> result(3, 0.0);
    if(shootingBack(q[1], q[0])) {
        result[0] = _F;
        result[1] = _M;
        result[2] = _zEnd;
    } else {
        RW_WARN("Backward beam equation not solved!");
    }
    
    return result;    
}

rw::math::Transform3D<> BeamJoint::getJointTransform(double F, double M, double s) const {
    std::cout << "getJointTransform: " << F << ", " << M << ", " << s << std::endl;
    rw::math::Transform3D<> result;
    // Solve the Euler-Bernoulli equation
    if(shooting(F, M)) {
        // Assume we're at the tip
        double as = _a.back(), zs = _z.back(), ys = _y.back();
        std::cout << "Result (y,z,a): " << ys << ", " << zs << ", " << as << std::endl;
        // List coordinate
        const double coord = (double)_n * s / _L;
        if((double)_n-coord > 0.01) {
            // Perform linear interpolation if not at the tip
            const unsigned int i = static_cast<unsigned int>(coord-1.0);
            const double diff = coord - (double)i;
            as = _a[i] + diff * (_a[i+1] - _a[i]);
            zs = _z[i] + diff * (_z[i+1] - _z[i]);
            ys = _y[i] + diff * (_y[i+1] - _y[i]);
        }

        std::cout << "Result again: " << ys << ", " << zs << ", " << as << std::endl;
        
        // Position
        const rw::math::Vector3D<> P(0.0, ys, zs);

        const double ca = std::cos(-as), sa = std::sin(-as);
        const rw::math::Rotation3D<> R(1.0, 0.0, 0.0, 0.0, ca, -sa, 0.0, sa, ca);

        result = _transform * rw::math::Transform3D<>(P, R);
    } else {
        RW_WARN("Beam equation not solved!");
    }

    return result;
}

bool BeamJoint::shooting(double F, double M) const {
    Shoot shoot(0.0, _L, _n, F, M, _E*_I, _L); 
    // Shoot by alternating the initial condition for kappa
    const double kappa_max = (std::abs(M) + std::abs(F)*_L) / _E*_I;
    try {
        /*const double kappa_sol = */zbrent<Shoot>(shoot, -kappa_max, kappa_max, 0.001);
    } catch(...) {
        // TODO
        return false;
    }

    const int start = shoot._out.count-_n;
    _a.assign(&shoot._out.ysave[1][start], &shoot._out.ysave[1][start] + _n);
    _z.assign(&shoot._out.ysave[2][start], &shoot._out.ysave[2][start] + _n);
    _y.assign(&shoot._out.ysave[3][start], &shoot._out.ysave[3][start] + _n);

    return true;
}

bool BeamJoint::shootingBack(double a, double y) const {
    ShootBack shoot(0.0, _L, _n, a, y, _E*_I, _L);
    VecDoub result(3, 0.0);
    // Instantiate simplex
    Amoeba am(0.00001);
    MatDoub simp(4, 3);
    simp[0][0] = 100; simp[0][1] = 30; simp[0][2] = _L;
    simp[1][0] = 100; simp[1][1] = -30; simp[1][2] = _L;
    simp[2][0] = -100; simp[2][1] = 30; simp[2][2] = _L;
    simp[3][0] = -100; simp[3][1] = -30; simp[3][2] = 0.9*_L;
    try {
        result = am.minimize<ShootBack>(simp, shoot);
    } catch(...) {
        // TODO
        return false;
    }

    _F = result[0];
    _M = result[1];
    _zEnd = result[2];
    
    return true;
}

/* 
 * joint: the transform of this joint (beam tip) seen from external frame
 * tcp: the transform of TCP seen from external frame
 */ 
void BeamJoint::getJacobian(size_t row,
        size_t col,
        const rw::math::Transform3D<>& joint,
        const rw::math::Transform3D<>& tcp,
        const rw::kinematics::State& state,
        rw::math::Jacobian& jacobian) const
{
    // Get configuration
    const rw::math::Q q(2, getData(state));
    // Find z-coordinate
    const std::vector<double> parms = solveParameters(q);
    // For readability
    const double &y = q[0], &a = q[1], &z = parms[2];
    // z-derivatives
    const double dzdy = ( std::abs(y) > 0.000001 ? (z - _L) / y : 0.0 ),
            dzda = ( std::abs(a) > 0.000001 ? (z - _L) / a : 0.0 );
    // Position columns
    rw::math::Vector3D<> py(0.0, 1.0, dzdy), pa(0.0, 0.0, dzda);
    // Rotation columns
    rw::math::Vector3D<> ry(0.0, 0.0, 0.0), ra(1.0, 0.0, 0.0);

    // Rotate into external base
    const rw::math::Rotation3D<>& R = joint.R();
    py = R * py;
    pa = R * pa;
    ry = R * ry;
    ra = R * ra;

    // Transform TCP
    const rw::math::Vector3D<> p = tcp.P() - joint.P();
    // Cross-product matrix
    const rw::math::Rotation3D<> S(0.0, p[2], -p[1], -p[2], 0.0, p[0], p[1], -p[0], 0.0);
    // Transformed positions
    py += S * ry;
    pa += S * ra;

    // y-column
    jacobian.addPosition(py, row, col);
    jacobian.addRotation(ry, row, col);
    // a-column
    jacobian.addPosition(pa, row, col+1);
    jacobian.addRotation(ra, row, col+1);

    std::cout << "-- getJacobian" << std::endl <<
            "\t" << q << std::endl <<
            "\t" << jacobian << std::endl;
}

}}
