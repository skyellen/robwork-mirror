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


#include "EAA.hpp"
#include "Rotation3D.hpp"
#include "Constants.hpp"

using namespace rw::math;

namespace
{
    template<class T> Vector3D<T> angleAxis(const Rotation3D<T>& R)
    {
        typedef Vector3D<T> V;

        const T epsilon = static_cast<T>(1e-12);

        T cos_theta = (T)0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);

        // Numerical rounding errors force us to make this check:
        if (cos_theta > 1) cos_theta = 1;
        else if (cos_theta < -1) cos_theta = -1;

        // ... because otherwise this often yields NaN:
        const T angle = acos(cos_theta);


        const V axis(R(2, 1) - R(1, 2),
					 R(0, 2) - R(2, 0),
					 R(1, 0) - R(0, 1));

        // angle approximately 0 degree
        if (fabs(angle) < epsilon) {
            return 0.5 * axis;
        }
        // 180 degree (is this formula always correct in this case?)
        else if (fabs(angle - Pi) < epsilon) {
			T len = axis.norm2();
			//return (axis / len)*(Pi/(1+len));
			return axis / (Pi/(len*(1+len)) );
			//return (T)Pi * V(
            //    sqrt((T)0.5 * (R(0, 0) + (T)1.0)),
            //    sqrt((T)0.5 * (R(1, 1) + (T)1.0)),
            //    sqrt((T)0.5 * (R(2, 2) + (T)1.0)));
		} else {
			return normalize(axis)*angle;
		}

       /* const V dir(
            R(2, 1) - R(1, 2),
            R(0, 2) - R(2, 0),
            R(1, 0) - R(0, 1));

        // For small, but non-zero angles so that angle ~= sin(angle) we have:
        //   angle / (2.0 * sin(angle)) ~= 0.5
        // Is this an optimization for speed or precision?
        if (fabs(angle) < 0.0001)
            return 0.5 * dir;
        else
            return (angle / (2 * sin(angle))) * dir;
			*/
    }
}

template<class T>
EAA<T>::EAA(const Rotation3D<T>& R)
    : _eaa(angleAxis(R))
{}

template<class T>
const Rotation3D<T> EAA<T>::toRotation3D() const
{
    T theta = angle();
    T ca = cos(theta);
    T sa = sin(theta);
    T va = 1-ca;

    Vector3D<T> k = axis();
    T kx = k[0];
    T ky = k[1];
    T kz = k[2];

    return Rotation3D<T>(
        kx * kx * va + ca , kx * ky * va - kz * sa , kx * kz * va + ky * sa,
        kx * ky * va + kz * sa, ky * ky * va + ca, ky * kz * va - kx * sa,
        kx * kz * va - ky * sa, ky * kz * va + kx * sa, kz * kz * va + ca);
}

// some explicit template specifications
template class EAA<double>;
template class EAA<float>;
