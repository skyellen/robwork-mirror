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

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/kinematics/State.hpp>

#include "nr3.h"
#include "svd.h"
//#include "ludcmp.h"

namespace rw { namespace models {

BeamJoint::BeamJoint(const std::string& name, const rw::math::Transform3D<>& transform) :
                     Joint(name, 2), _transform(transform), _L(1), _E(1), _I(1)
{
  
}

BeamJoint::~BeamJoint()
{
}

rw::math::Transform3D<> BeamJoint::getJointTransform(const rw::math::Q& q) const
{
  const std::vector<double> parms = solveParameters(q);
  
  const double &F = parms[0], &M = parms[1], &z = parms[2];

  return getJointTransform(F, M, z);
}

std::vector<double> BeamJoint::solveParameters(const rw::math::Q& q) const
{
  // Control input (translation and angle)
  const double &y = q[0], &a = q[1];
  // User function for evaluating the objective function and the Jacobian
  struct usrfun {
    double _y, _a;
    double _L, _E, _I;
    VecDoub _f;
    MatDoub _jac;
    
    usrfun(double y, double a, double L, double E, double I) : _y(y), _a(a), _L(L), _E(E), _I(I), _f(3), _jac(3,3) {}
  
    const VecDoub& f(const VecDoub& x) {
      const double &F = x[0], &M = x[1], &z = x[2];
      _f[0] = _y - ( ((z - 3.0*_L)*F + 3*M)*z*z ) / (6.0*_E*_I);
      _f[1] = _a - std::atan( ( ((z - 2.0*_L)*F + 2.0*M)*z ) / (2.0*_E*_I) );
      _f[2] = z*z + _y*_y - _L*_L;
      
      return _f;
    }
    
    const MatDoub& jac(const VecDoub& x) {
      const double &F = x[0], &M = x[1], &z = x[2];
      _jac[0][0] = -((z*z*(-3.0*_L + z))/(6*_E*_I));
      _jac[0][1] = -z*z / (2.0*_E*_I);
      _jac[0][2] = -((z*(2*M + F*(-2.0*_L + z)))/(2*_E*_I));
      
      const double parsq = (2.0*M + F*(-2.0*_L + z)) * (2.0*M + F*(-2.0*_L + z));
      const double EEII = _E*_E*_I*_I;
      _jac[1][0] = -((2*_E*_I*z*(-2.0*_L + z)) / (4.0*EEII + z*z*parsq));
      _jac[1][1] = -((4.0*_E*_I*z)/(4.0*EEII + z*z*parsq));
      _jac[1][2] = -((4.0*_E*_I*(M + F*(-_L + z)))/(4.0*EEII + z*z*parsq));
      
      _jac[2][0] = 0.0;
      _jac[2][1] = 0.0;
      _jac[2][2] = 2.0*z;
      
      return _jac;
    }
  } fjac(y, a, _L, _E, _I);
  
  // Starting guess
  const double F = 0.0, M = 0.0, z = _L;
  // Solution vector
  VecDoub x(3);
  x[0] = F; x[1] = M; x[2] = z;
  // Trials
  const int ntrial = 10;
  // Tolerances
  const double tolf = 0.000001, tolx = tolf;
  for(int i = 1; i <= ntrial; ++i) {
    // Function and Jacobian at x
    const VecDoub& f = fjac.f(x);
    //std::cout << "f: " << f[0] << " " << f[1] << " " << f[2] << std::endl;
    const MatDoub& j = fjac.jac(x);
    /*
    std::cout << "j: " << std::endl <<
        "\t" << j[0][0] << " " << j[0][1] << " " << j[0][2] << std::endl <<
        "\t" << j[1][0] << " " << j[1][1] << " " << j[1][2] << std::endl <<
        "\t" << j[2][0] << " " << j[2][1] << " " << j[2][2] << std::endl;
    */
    // Function error
    double errf = std::abs(f[0]) + std::abs(f[1]) + std::abs(f[2]);
    // Function convergence
    if(errf < tolf)
      break;
    // Right-hand side
    VecDoub b(3);
    b[0] = -f[0]; b[1] = -f[1]; b[2] = -f[2];
    // Correction for x
    VecDoub dx(3);
    // Solve
    try {
      SVD svd(j);
      svd.solve(b, dx);
    } catch(int e) {
      RW_WARN("Exception caught from SVD - inaccurate solution for beam joint expected!");
      break;
    }
    // Solution vector error
    double errx = std::abs(dx[0]) + std::abs(dx[1]) + std::abs(dx[2]);
    // Solution vector correction
    x[0] += dx[0]; x[1] += dx[1]; x[2] += dx[2];
    // Solution vector convergence
    if(errx < tolx)
      break;
  }
  
  // Solution (force F, moment M and projected length z)
  std::vector<double> sol(3);
  sol[0] = x[0]; sol[1] = x[1]; sol[2] = x[2];
  
  return sol;
}

rw::math::Transform3D<> BeamJoint::getJointTransform(double F, double M, double z) const
{
  // Deflection at z with F and M acting at the tip
  const double y = deflection(F, M, z); 
  
  // Position
  const rw::math::Vector3D<> P(0.0, y, z);
  
  // Rotation
  const double a = angle(F, M, z);
  const double ca = std::cos(a), sa = std::sin(a);
  const rw::math::Rotation3D<> R(1.0, 0.0, 0.0, 0.0, ca, -sa, 0.0, sa, ca);
  
  return _transform * rw::math::Transform3D<>(P, R);
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
}

void BeamJoint::setBounds(const std::pair<const math::Q, const math::Q>& bounds)
{
  if(bounds.first[0] < -45.0*rw::math::Deg2Rad || bounds.second[0] > 45.0*rw::math::Deg2Rad ||
     bounds.first[1] < -45.0*rw::math::Deg2Rad || bounds.second[1] > 45.0*rw::math::Deg2Rad)
    RW_THROW("Beam joint bound out of range - must be between -45 and 45 degrees.");
  
  Joint::setBounds(bounds);
}

}}
