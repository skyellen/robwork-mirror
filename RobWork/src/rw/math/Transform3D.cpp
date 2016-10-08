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


#include "Transform3D.hpp"

#include <math.h>

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "Math.hpp"

using namespace rw::common;
using namespace rw::math;

template<class T>
const Transform3D<T> Transform3D<T>::DH(T alpha, T a, T d, T theta)
{
  return Transform3D(
      Vector3D<T>(a * cos(theta), a * sin(theta), d),
      Rotation3D<T>(
          cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
          sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha),
          0, sin(alpha), cos(alpha)));
}

template<class T>
const Transform3D<T> Transform3D<T>::DHHGP(T alpha, T a, T beta, T b)
{
       return Transform3D(
               Vector3D<T>(a * cos(beta), b, - a * sin(beta)),
               Rotation3D<T>(
                   cos(beta), sin(alpha) * sin(beta), cos(alpha) * sin(beta),
                   0, cos(alpha), -sin(alpha),
                   -sin(beta), sin(alpha) * cos(beta), cos(alpha) * cos(beta)));
}


template<class T>
const Transform3D<T> Transform3D<T>::craigDH(T alpha, T a, T d, T theta)
{
    return Transform3D(
        Vector3D<T>(a, -sin(alpha) * d, cos(alpha) * d),
        Rotation3D<T>(
            cos(theta),-sin(theta), 0,
            sin(theta)*cos(alpha), cos(theta) * cos(alpha), -sin(alpha),
            sin(theta)*sin(alpha), cos(theta) * sin(alpha), cos(alpha)));
}

template<class T>
typename Transform3D<T>::EigenMatrix4x4 Transform3D<T>::e() const
{
    EigenMatrix4x4 matrix;
    matrix.block(0, 0, 3, 3) = _R.e();
    matrix.block(0, 3, 3, 1) = _d.e();
    matrix(3, 0) = matrix(3, 1) = matrix(3, 2) = 0.0;
    matrix(3, 3) = 1.0;
    return matrix;
}

// Explicit template instantiations.
template class rw::math::Transform3D<double>;
template class rw::math::Transform3D<float>;


namespace rw{ namespace common { namespace serialization {

    template<class T>
    void writeImpl(const Transform3D<T>& tmp, OutputArchive& oar, const std::string& id){
        std::vector<double> data = Math::toStdVector(tmp, 3, 4);
        oar.write( data , id );
    }

    template<class T>
    void readImpl(Transform3D<T>& tmp, InputArchive& iar, const std::string& id){
        std::vector<T> data;
        iar.read(data, id);
        Math::fromStdVectorToMat(data, tmp, 3, 4 );
    }

    // we need these to explicitly instantiate these functions
    template<> void write(const Transform3D<double>& tmp, OutputArchive& oar, const std::string& id ) { writeImpl(tmp,oar,id); }
    template<> void write(const Transform3D<float>& tmp, OutputArchive& oar, const std::string& id ) { writeImpl(tmp,oar,id); }
    template<> void read(Transform3D<double>& tmp, InputArchive& iar, const std::string& id) { readImpl(tmp,iar,id); }
    template<> void read(Transform3D<float>& tmp, InputArchive& iar, const std::string& id) { readImpl(tmp,iar,id); }

}}}
