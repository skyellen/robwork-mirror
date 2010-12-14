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

namespace rw { namespace models {

BeamJoint::BeamJoint(const std::string& name, const rw::math::Transform3D<>& transform) :
                     Joint(name, 1), _transform(transform),
                     _L(1), _E(1), _I(1)
{
}

BeamJoint::~BeamJoint()
{
}

rw::math::Transform3D<> BeamJoint::getJointTransform(double F) const
{
  // z-coordinate of the tip under deflection
  const double zF = projectedLength(F);
  
  // Deflection/slope at z = zF with F acting at the tip
  const double y = deflection(F, zF);
  
  // Angle from z-axis to tip around x-axis
  const double a = angle(F, zF);
  
  // Position of the tip
  const rw::math::Vector3D<> P(0.0, y, zF);
  
  // Rotation of the tip around x-axis
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
  // Current force input
  const double F = getData(state)[0];
  
  // z-coordinate of the tip under deflection
  const double z = projectedLength(F);
  
  /*
   * Linear/angular joint velocity in local frame (beam tip)
   */
  // dy/dF relative to beam base
  const double dydF = (3.0*_L - z)*z*z / (6.0*_E*_I);
  // dp at beam tip
  const double a = -std::atan(slope(F, z));
  rw::math::Vector3D<> dp(0.0, std::cos(a)*dydF, -std::sin(a)*dydF);
  
  // Analytical expression for da/dF = d/dF( -atan(y'(z)) )
  const double par = z - 2.0*_L;
  const double dadF = par*2.0*_E*_I*z / ( 4.0*_E*_E*_I*_I + F*F*z*z*par*par );
  // Rotation of the tip around x-axis
  rw::math::Vector3D<> dv(dadF, 0.0, 0.0);
  
  /*
   * Linear/angular joint velocity in external frame
   */
  // Change of reference frame for the local Jacobian
  const rw::math::Rotation3D<>& worldRjoint = joint.R();
  dp = worldRjoint * dp;
  dv = worldRjoint * dv;
  
  /*
   * Linear/angular joint velocity in external frame translated to TCP frame
   */
  const rw::math::Vector3D<> localPtcp = tcp.P() - joint.P();
  dp = dp + cross(dv, localPtcp);
  
  // Store result
  jacobian.addPosition(dp, row, col);
  jacobian.addRotation(dv, row, col);
}

double BeamJoint::projectedLength(double F, unsigned int ntrial, double toly, double tolz) const {
  // Deflection/slope at the tip
  const double y = deflection(F, _L), dy = slope(F, _L);
  // If force is too small to cause deflection above tolerance
  if(std::abs(y) < toly || std::abs(dy) < toly)
    return _L;
  
  // Step size
  const double h = 0.01 * _L;
  // z-interval where the root is
  double zLow = _L - h, zHigh = _L;
  // f-values for that interval
  double f = arcLength(F, zLow) - _L, fmid = arcLength(F, zHigh) - _L;
  // Trace backwards along z-axis to bracket the root between zLow and zHigh
  while(f*fmid >= 0.0 && zLow >= 0.0) {
    zLow -= h;
    f = arcLength(F, zLow) - _L;
  }
  // Move zHigh just above zLow
  zHigh = std::min(zLow + 2.0*h, zHigh);
  // Initialize f-value
  fmid = arcLength(F, zHigh) - _L;
  // Check if root is bracketed
  if(f*fmid >= 0.0)
    return -1.0;
  
  // Initialize z-start, z-change direction and midpoint for bisection
  double zF = zLow, dz = zHigh-zLow, zmid;
  if(f >= 0.0) {
    zF = zHigh;
    dz = -dz;
  }
  
  // Bisect
  for(unsigned int i = 0; i < ntrial; ++i) {
    // Halve the z-change
    dz *= 0.5;
    // Follow the z-change towards the root
    zmid = zF + dz;
    // Evaluate the f-value at the new midpoint
    fmid = arcLength(F, zmid) - _L;
    // Make sure that the f-value of zF stays below zero
    if(fmid <= 0.0)
      zF = zmid;
    // Check for convergence
    if(std::abs(dz) < tolz || std::abs(fmid) < toly)
      break;
  }

  return zF;
}

 double BeamJoint::arcLength(double F, double z) const {
  /*
   * The integrand used by this function
   */
  struct ArcLengthIntegrand {
    inline double operator()(double F, double z, const BeamJoint* bj) {
      const double dy = bj->slope(F, z);
      
      return std::sqrt(1.0 + dy*dy);
    }
  } arcLengthIntegrand;
  
  // Steps
  const unsigned int n = 100;
  // Step size
  const double h = z / (double)n;
  /*
   * Integration by Simpson's rule
   */
  // Result initialized by sum of endpoints
  double l = arcLengthIntegrand(F, 0, this) + arcLengthIntegrand(F, z, this);
  const double mulEven = 2.0, mulOdd = 4.0;
  for(unsigned int k = 1; k <= n-1; k+=2) {
    l += mulOdd * arcLengthIntegrand(F, (double)k*h, this);
    l += mulEven * arcLengthIntegrand(F, (double)(k+1)*h, this);
  }
  
  l *= h * 0.3333333333333333333333333333;
  
  return l;
}

}}
