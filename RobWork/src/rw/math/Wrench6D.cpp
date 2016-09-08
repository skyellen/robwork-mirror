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


#include "Wrench6D.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "Math.hpp"

using namespace rw::math;

template<class T>
Wrench6D<T>::Wrench6D(T vx, T vy, T vz, T wx, T wy, T wz) {
    _wrench[0] = vx;
    _wrench[1] = vy;
    _wrench[2] = vz;
    _wrench[3] = wx;
    _wrench[4] = wy;
    _wrench[5] = wz;
}

template<class T>
Wrench6D<T>::Wrench6D(const Vector3D<T>& force, const Vector3D<T>& torque) {
    _wrench[0] = force(0);
    _wrench[1] = force(1);
    _wrench[2] = force(2);
    _wrench[3] = torque(0);
    _wrench[4] = torque(1);
    _wrench[5] = torque(2);
}


template class Wrench6D<double>;
template class Wrench6D<float>;



namespace rw{ namespace common { namespace serialization {

    template<class T>
    void writeImpl(const rw::math::Wrench6D<T>& tmp, rw::common::OutputArchive& oar, const std::string& id){
        std::vector<double> data = rw::math::Math::toStdVector(tmp, 6);
        oar.write( data , id , "Wrench6D");
    }

    template<class T>
    void readImpl(rw::math::Wrench6D<T>& tmp, rw::common::InputArchive& iar, const std::string& id){
        std::vector<T> data;
        iar.read(data, id, "Wrench6D");
        rw::math::Math::fromStdVector(data, tmp );
    }

    // we need these to explicitly instantiate these functions
    template<> void write( const rw::math::Wrench6D<double>& tmp, rw::common::OutputArchive& oar, const std::string& id ){writeImpl(tmp,oar,id);}
    template<> void write( const rw::math::Wrench6D<float>& tmp, rw::common::OutputArchive& oar, const std::string& id ){writeImpl(tmp,oar,id);}
    template<> void read(rw::math::Wrench6D<double>& tmp, rw::common::InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);}
    template<> void read(rw::math::Wrench6D<float>& tmp, rw::common::InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);}

}}}
