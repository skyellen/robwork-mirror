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

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "Math.hpp"
#include "EAA.hpp"

using namespace rw::math;

namespace
{
    template<class T> Vector3D<T> angleAxis(const Rotation3D<T>& R)
    {
        typedef Vector3D<T> V;

		const T eps = static_cast<T>(1e-6);

        T cos_theta = (T)0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);

        // Numerical rounding errors force us to make this check:
        if (cos_theta > 1) cos_theta = 1;
        else if (cos_theta < -1) cos_theta = -1;

        // ... because otherwise this often yields NaN:
        const T angle = acos(cos_theta);
        const V axis(R(2, 1) - R(1, 2),
					 R(0, 2) - R(2, 0),
					 R(1, 0) - R(0, 1));

		if (fabs(angle) < eps) {   // Is angle close to 0 degree
			return 0.5 * axis;
		}

		if (fabs(angle - Pi) < eps) { // Is the angle close to 180 degree
			V magnitude(
					(T)Pi * sqrt((T)0.5 * (R(0, 0) + (T)1.0)),
					(T)Pi * sqrt((T)0.5 * (R(1, 1) + (T)1.0)),
					(T)Pi * sqrt((T)0.5 * (R(2, 2) + (T)1.0))
			);
			// Determine signs
			std::size_t maxInd = 0;
			if (magnitude[1] > magnitude[0]) maxInd = 1;
			if (magnitude[2] > magnitude[1]) maxInd = 2;
			if (maxInd == 0) {
				T v1v2 = R(0,1)+R(1,0);
				T v1v3 = R(0,2)+R(2,0);
				if (v1v2 < 0.)
					magnitude[1] = -magnitude[1];
				if (v1v3 < 0.)
					magnitude[2] = -magnitude[2];
			} else if (maxInd == 1) {
				T v1v2 = R(0,1)+R(1,0);
				T v2v3 = R(1,2)+R(2,1);
				if (v1v2 < 0.)
					magnitude[0] = -magnitude[0];
				if (v2v3 < 0.)
					magnitude[2] = -magnitude[2];
			} else if (maxInd == 2) {
				T v1v3 = R(0,2)+R(2,0);
				T v2v3 = R(1,2)+R(2,1);
				if (v1v3 < 0.)
					magnitude[0] = -magnitude[0];
				if (v2v3 < 0.)
					magnitude[1] = -magnitude[1];
			}
			return magnitude;
		} 

		return normalize(axis)*angle;
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
template class rw::math::EAA<double>;
template class rw::math::EAA<float>;

template<>
void rw::common::serialization::write(const EAA<double>& tmp, rw::common::OutputArchive& oar, const std::string& id)
{
    oar.write( rw::math::Math::toStdVector(tmp, (int)tmp.size()), id , "EAA");
}
template<>
void rw::common::serialization::read(EAA<double>& tmp, rw::common::InputArchive& iar, const std::string& id){
    std::vector<double> arr;
    iar.read(arr, id, "EAA");
    rw::math::Math::fromStdVector(arr, tmp);
}
template<>
void rw::common::serialization::write(const EAA<float>& tmp, rw::common::OutputArchive& oar, const std::string& id)
{
    oar.write( rw::math::Math::toStdVector(tmp, (int)tmp.size()), id , "EAA");
}
template<>
void rw::common::serialization::read(EAA<float>& tmp, rw::common::InputArchive& iar, const std::string& id){
    std::vector<float> arr;
    iar.read(arr, id, "EAA");
    rw::math::Math::fromStdVector(arr, tmp);
}
