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


#include "InertiaMatrix.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "Math.hpp"

using namespace rw::math;


// some explicit template specifications
template class InertiaMatrix<double>;
template class InertiaMatrix<float>;



namespace rw{ namespace common { namespace serialization {

    template<class T>
    void write(const rw::math::InertiaMatrix<T>& tmp, rw::common::OutputArchive& oar, const std::string& id){
        std::vector<double> data = rw::math::Math::toStdVector(tmp, 3, 3);
        oar.write( data , id );
    }

    template<class T>
    void read(rw::math::InertiaMatrix<T>& tmp, rw::common::InputArchive& iar, const std::string& id){
        std::vector<double> data;
        iar.read(data, id);
        rw::math::Math::fromStdVectorToMat(data, tmp, 3, 3 );
    }

    // we need these to explicitly instantiate these functions
    template void write<double>( const rw::math::InertiaMatrix<double>& tmp, rw::common::OutputArchive& oar, const std::string& id );
    template void write<float>( const rw::math::InertiaMatrix<float>& tmp, rw::common::OutputArchive& oar, const std::string& id );
    template void read<double>(rw::math::InertiaMatrix<double>& tmp, rw::common::InputArchive& iar, const std::string& id);
    template void read<float>(rw::math::InertiaMatrix<float>& tmp, rw::common::InputArchive& iar, const std::string& id);

}}}
