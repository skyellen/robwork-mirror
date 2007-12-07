/*********************************************************************
 * RobWork Version 0.2
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

#include "Transform3D.hpp"

#include <cassert>
#include <math.h>

using namespace rw::math;

template<class T>
Transform3D<T> Transform3D<T>::DH(T alpha, T a, T d, T theta){
  return Transform3D(
      Vector3D<T>(a*cos(theta), a*sin(theta), d),
      Rotation3D<T>(
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                 0,             sin(alpha),                      d
        )
      );
}

template<class T>
Transform3D<T> Transform3D<T>::CraigDH(T alpha, T a, T d, T theta){
    return Transform3D(
            Vector3D<T>(a, -sin(alpha) * d, cos(alpha) * d),
            Rotation3D<T>(
                cos(theta),-sin(theta), 0,
                sin(theta)*cos(alpha), cos(theta) * cos(alpha), -sin(alpha),
                sin(theta)*sin(alpha), cos(theta) * sin(alpha), cos(alpha)
            )
    );
    	
}
/*
template<class T>
const Transform3D<T>& Transform3D<T>::Identity() {
    static const Transform3D id(
        Vector3D<T>(0, 0, 0),
        Rotation3D<T>::Identity());
    return id;
}
*/

// explicit template instantiations
template class Transform3D<double>;
template class Transform3D<float>;
