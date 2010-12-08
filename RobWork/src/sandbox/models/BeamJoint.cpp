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

#include <rw/math/EAA.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

BeamJoint::BeamJoint(const std::string& name, const Transform3D<>& transform) :
                     Joint(name, 1), _transform(transform),
                     _L(1), _E(1), _I(1)
{
}

BeamJoint::~BeamJoint()
{
}

Transform3D<> BeamJoint::getJointTransform(double F) const
{
    //std::cout << "getJointTransform: F=" << F << ", L=" << _L << std::endl;
    double zF = projectedLength(F);
    if(zF < 0.0)
      zF = _L;
    
    //std::cout << "zF = " << zF << std::endl;
    // Deflection/slope at z = L with F acting at z = l
    const double y = deflection(F, _L);
    const double dy = slope(F, _L);
    const double a = std::atan(dy);
    
    const Vector3D<> P(0.0, y, zF);
    const Rotation3D<> R = RPY<>(0.0, 0.0, -a).toRotation3D();
    
    return _transform * Transform3D<>(P, R);
}


void BeamJoint::getJacobian(size_t row, size_t col, const Transform3D<>& joint, const Transform3D<>& tcp, Jacobian& jacobian) const
{
  /* joint: the transform of this joint seen from external frame
   * tcp: the transform of TCP seen from external frame
   */
  const Vector3D<> axis = joint.R().getCol(2);
  const Vector3D<> p = cross(axis, tcp.P() - joint.P());

  jacobian.addPosition(p, row, col);
  jacobian.addRotation(axis,row, col);
}

double BeamJoint::projectedLength(double F, double tolz, double toldz, bool debug) const {
  // If force is too small to cause deflection above tolerance
  const double y = deflection(F, _L), dy = slope(F, _L);
  if(std::abs(y) < tolz || std::abs(dy) < toldz) {
    if(debug) std::cout << "Deflection/slope at z=L close to zero" << std::endl;
    if(debug) std::cout << "\tdeflection: " << y << std::endl <<
                            "\tslope: " << dy << std::endl;
    return _L;
  }
  // Step size
  const double h = 0.01 * _L;
  // Trace backwards along z-axis to bracket the root between zLow and zHigh
  double zLow = _L * (1.0 - h), zHigh = _L;
  double f = arcLength(F, zLow) - _L;
  double fmid = arcLength(F, zHigh) - _L;
  while(f*fmid >= 0.0 && zLow > 0.0) {
    zLow -= h * _L;
    f = arcLength(F, zLow) - _L;
  }
  zHigh = std::min(zLow + 2.0*h, zHigh);
  fmid = arcLength(F, zHigh) - _L;
  if(debug) std::cout << "(zLow, zHigh): (" << zLow << ", " << zHigh << ")" << std::endl;
  if(debug) std::cout << "(f(zLow), f(zHigh)): (" << f << ", " << fmid << ")" << std::endl;
  if(f*fmid >= 0.0) {
    if(debug) std::cout << "Root not bracketed!" << std::endl;
    return -1.0;
  }
  double dz, zmid, zF;
  dz = zHigh-zLow;
  zF = zLow;
  if(f >= 0.0) {
    dz = -dz;
    zF = zHigh;
  }
  
  // Bisect
  bool success = false;
  const unsigned int ntrial = 20;
  for(unsigned int i = 0; i < ntrial; ++i) {
    if(debug) std::cout << "Iteration " << i+1 << std::endl;
    dz *= 0.5;
    zmid = zF + dz;
    fmid = arcLength(F, zmid) - _L;
    if(debug) std::cout << "\tzmid= (" << zmid << ")" << std::endl;
    if(debug) std::cout << "\tfmid= (" << fmid << ")" << std::endl;
    if(fmid <= 0.0)
      zF = zmid;
    if(std::abs(dz) < toldz || fmid == 0.0) {
      if(debug) std::cout << "SUCCESS: zF = " << zF << std::endl;
      success = true;
      break;
    }
  }
  
  if(!success)
    zF = -1;

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
  const double n = 100.0;
  // Step size
  const double h = z / n;
  // Result
  double l = arcLengthIntegrand(F, 0, this) + arcLengthIntegrand(F, z, this);
  bool even = false;
  const double mulEven = 2.0, mulOdd = 4.0;
  for(unsigned int k = 1; k <= n-1; ++k, even = !even) {
    l += even ? mulEven * arcLengthIntegrand(F, (double)k*h, this) : 
                mulOdd * arcLengthIntegrand(F, (double)k*h, this);
  }
  const double oneoverthree = 0.3333333333333333333333333333;
  l *= h * oneoverthree;
  
  return l;
}