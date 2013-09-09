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


#include "Quaternion.hpp"
#include "Math.hpp"
#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::math;

template class Quaternion<double>;
template class Quaternion<float>;


void rw::common::serialization::write(const Quaternion<double>& tmp, rw::common::OutputArchive& oar, const std::string& id)
{
    oar.write( rw::math::Math::toStdVector(tmp, tmp.size()), id );
}

void rw::common::serialization::read(Quaternion<double>& tmp, rw::common::InputArchive& iar, const std::string& id){
    std::vector<double> arr;
    iar.read(arr, id);
    rw::math::Math::fromStdVector(arr, tmp);
}

void rw::common::serialization::write(const Quaternion<float>& tmp, rw::common::OutputArchive& oar, const std::string& id)
{
    oar.write( rw::math::Math::toStdVector(tmp, tmp.size()), id );
}

void rw::common::serialization::read(Quaternion<float>& tmp, rw::common::InputArchive& iar, const std::string& id){
    std::vector<double> arr;
    iar.read(arr, id);
    rw::math::Math::fromStdVector(arr, tmp);
}
