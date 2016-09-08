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

#include "Rotation2D.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "Math.hpp"

using namespace rw::common;
using namespace rw::math;

// Explicit template specifications.
template class Rotation2D<double>;
template class Rotation2D<float>;


namespace rw{ namespace common { namespace serialization {

    template<class T>
    void writeImpl(const Rotation2D<T>& tmp, OutputArchive& oar, const std::string& id){
        std::vector<double> data = Math::toStdVector(tmp, 2, 2);
        oar.write( data , id , "Rotation2D");
    }

    template<class T>
    void readImpl(Rotation2D<T>& tmp, InputArchive& iar, const std::string& id){
        std::vector<T> data;
        iar.read(data, id, "Rotation2D");
        Math::fromStdVectorToMat(data, tmp, 2, 2 );
    }

    // we need these to explicitly instantiate these functions
    template<> void write( const Rotation2D<double>& tmp, OutputArchive& oar, const std::string& id ){writeImpl(tmp,oar,id);}
    template<> void write( const Rotation2D<float>& tmp, OutputArchive& oar, const std::string& id ){writeImpl(tmp,oar,id);}
    template<> void read(Rotation2D<double>& tmp, InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);}
    template<> void read(Rotation2D<float>& tmp, InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);}

}}}
