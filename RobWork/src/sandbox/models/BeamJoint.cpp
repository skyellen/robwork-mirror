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

namespace rw { namespace models {

BeamJoint::BeamJoint(const std::string& name, const rw::math::Transform3D<>& transform) :
                     Joint(name, 1), _transform(transform), _L(1)
{
}

BeamJoint::~BeamJoint()
{
}

rw::math::Transform3D<> BeamJoint::getJointTransform(const rw::math::Q& a)
{
  return getJointTransform(a[0]);
}

rw::math::Transform3D<> BeamJoint::getJointTransform(double a) const
{
  struct usrfun {
    VecDoub _f;
    MatDoub _jac;
    
    usrfun() : _f(2), _jac(2,2) {}
  
    const VecDoub& f(const VecDoub& x, double a, double L) {
      const double& y = x[0], z = x[1];
      _f[0] = y + std::tan(a) * (z*z - 3.0*L*z) / (3.0*z - 6.0*L);
      _f[1] = z - ( std::abs(y) > L ? 0.0 : std::sqrt(L*L - y*y) );
      
      return _f;
    }
    
    const MatDoub& jac(const VecDoub& x, double a, double L) {
      const double& y = x[0], z = x[1];
      _jac[0][0] = 1.0;
      _jac[0][1] = std::tan(a) * ( 1.0 + 2.0*L*L / ((z - 2.0*L)*(z - 2.0*L)) ) / 3.0;
      _jac[1][0] = y / ( std::abs(y) > L ? 0.0 : std::sqrt(L*L - y*y) );
      _jac[1][1] = 1.0;
      
      return _jac;
    }
  } fjac;
  
  // Starting guess
  double y = 0.1*_L, z = 0.9*_L;
  // Solution vector
  VecDoub x(2); x[0] = y; x[1] = z;
  // Trials
  const int ntrial = 10;
  // Tolerances
  const double tolf = 0.000001, tolx = tolf;
  for(int i = 1; i <= ntrial; ++i) {
    // Function and Jacobian at x
    const VecDoub& f = fjac.f(x, a, _L);
    const MatDoub& j = fjac.jac(x, a, _L);
    // Function error
    double errf = std::abs(f[0]) + std::abs(f[1]);
    // Function convergence
    if(errf < tolf)
      break;
    // Right-hand side
    VecDoub b(2);
    b[0] = -f[0]; b[1] = -f[1];
    // Correction for x
    VecDoub dx(2);
    // Solve using SVD
    try {
      SVD svd(j);
      svd.solve(b, dx);
    } catch(int e) {
      break;
    }
    // Solution vector error
    double errx = std::abs(dx[0]) + std::abs(dx[1]);
    // Solution vector correction
    x[0] += dx[0]; x[1] += dx[1];
    // Solution vector convergence
    if(errx < tolx)
      break;    
  }
  
  y = x[0]; z = x[1];
  
  // Position
  const rw::math::Vector3D<> P(0.0, y, z);
  
  // Rotation
  const double ca = std::cos(a), sa = std::sin(a);
  const rw::math::Rotation3D<> R(1.0, 0.0, 0.0, 0.0, ca, -sa, 0.0, sa, ca);
  
  return _transform * rw::math::Transform3D<>(P, R);
}

rw::math::Transform3D<> BeamJoint::getJointTransform(double a, double z) const
{
  
  // Deflection at z with F acting at the tip
  const double y = deflection(a, z);
  
  // Position
  const rw::math::Vector3D<> P(0.0, y, z);
  
  // Rotation
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
  if(bounds.first[0] < -45.0*rw::math::Deg2Rad || bounds.second[0] > 45.0*rw::math::Deg2Rad)
    RW_THROW("Beam joint bound out of range - must be between -45 and 45 degrees.");
  
  Joint::setBounds(bounds);
}

}}
