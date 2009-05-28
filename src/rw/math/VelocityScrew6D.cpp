/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "VelocityScrew6D.hpp"

using namespace rw::math;

template<class T>
VelocityScrew6D<T>::VelocityScrew6D(T vx, T vy, T vz, T wx, T wy, T wz) : _screw(6){
    _screw[0] = vx;
    _screw[1] = vy;
    _screw[2] = vz;
    _screw[3] = wx;
    _screw[4] = wy;
    _screw[5] = wz;
}

template<class T>
VelocityScrew6D<T>::VelocityScrew6D(const Transform3D<T>& transform) : _screw(6) {
  //      const Vector3D<T>& v = transform.P();
  _screw(0) = transform(0,3);
  _screw(1) = transform(1,3);
  _screw(2) = transform(2,3);

  EAA<T> eaa(transform.R());

  _screw(3) = eaa(0);
  _screw(4) = eaa(1);
  _screw(5) = eaa(2);

  /*_screw(3) = (T)0.5*(transform(2,1)-transform(1,2));
  _screw(4) = (T)0.5*(transform(0,2)-transform(2,0));
  _screw(5) = (T)0.5*(transform(1,0)-transform(0,1));*/
}

template<class T>
VelocityScrew6D<T>::VelocityScrew6D(const Vector3D<T>& linear, const EAA<T>& angular) : _screw(6){
  _screw(0) = linear(0);
  _screw(1) = linear(1);
  _screw(2) = linear(2);
  _screw(3) = angular[0];
  _screw(4) = angular[1];
  _screw(5) = angular[2];
}

template class VelocityScrew6D<double>;
template class VelocityScrew6D<float>;


