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

#include "Rotation3D.hpp"
#include "Rotation3DVector.hpp"

using namespace rw::math;

template<class T>
Rotation3D<T>::Rotation3D(const Rotation3DVector<T>& r) :
    _matrix(boost::numeric::ublas::identity_matrix<T>(3))
{
    (*this) = r.toRotation3D( );
}

// some explicit template specifications
template class Rotation3D<double>;
template class Rotation3D<float>;
