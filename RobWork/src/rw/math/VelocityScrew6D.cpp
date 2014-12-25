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


#include "VelocityScrew6D.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "Math.hpp"

using namespace rw::math;

template<class T>
VelocityScrew6D<T>::VelocityScrew6D(T vx, T vy, T vz, T wx, T wy, T wz) {
    _screw[0] = vx;
    _screw[1] = vy;
    _screw[2] = vz;
    _screw[3] = wx;
    _screw[4] = wy;
    _screw[5] = wz;
}

template<class T>
VelocityScrew6D<T>::VelocityScrew6D(const Transform3D<T>& transform) {
  //      const Vector3D<T>& v = transform.P();
  _screw[0] = transform(0,3);
  _screw[1] = transform(1,3);
  _screw[2] = transform(2,3);

  EAA<T> eaa(transform.R());

  _screw[3] = eaa(0);
  _screw[4] = eaa(1);
  _screw[5] = eaa(2);

  /*_screw(3) = (T)0.5*(transform(2,1)-transform(1,2));
  _screw(4) = (T)0.5*(transform(0,2)-transform(2,0));
  _screw(5) = (T)0.5*(transform(1,0)-transform(0,1));*/
}

template<class T>
VelocityScrew6D<T>::VelocityScrew6D(const Vector3D<T>& linear, const EAA<T>& angular) {
  _screw[0] = linear(0);
  _screw[1] = linear(1);
  _screw[2] = linear(2);
  _screw[3] = angular[0];
  _screw[4] = angular[1];
  _screw[5] = angular[2];
}

template class VelocityScrew6D<double>;
template class VelocityScrew6D<float>;



namespace rw{ namespace common { namespace serialization {

    template<class T>
    void writeImpl(const rw::math::VelocityScrew6D<T>& tmp, rw::common::OutputArchive& oar, const std::string& id){
        std::vector<double> data = rw::math::Math::toStdVector(tmp, 6);
        oar.write( data , id , "Twist6D");
    }

    template<class T>
    void readImpl(rw::math::VelocityScrew6D<T>& tmp, rw::common::InputArchive& iar, const std::string& id){
        std::vector<T> data;
        iar.read(data, id, "Twist6D");
        rw::math::Math::fromStdVector(data, tmp );
    }

    // we need these to explicitly instantiate these functions
    template<> void write( const rw::math::VelocityScrew6D<double>& tmp, rw::common::OutputArchive& oar, const std::string& id ){writeImpl(tmp,oar,id);};
    template<> void write( const rw::math::VelocityScrew6D<float>& tmp, rw::common::OutputArchive& oar, const std::string& id ){writeImpl(tmp,oar,id);};
    template<> void read(rw::math::VelocityScrew6D<double>& tmp, rw::common::InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);};
    template<> void read(rw::math::VelocityScrew6D<float>& tmp, rw::common::InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);};

}}}
