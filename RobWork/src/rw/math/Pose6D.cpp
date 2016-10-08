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


#include "Pose6D.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::math;

template class rw::math::Pose6D<double>;
template class rw::math::Pose6D<float>;


namespace rw{ namespace common {namespace serialization {

    template<class T>
    void writeImpl(const rw::math::Pose6D<T>& tmp, rw::common::OutputArchive& oar, const std::string& id){
        oar.writeEnterScope(id, "Pose6D");
        oar.write( tmp.getPos() , "pos" );
        oar.write( tmp.getEAA() , "eaa" );
        oar.writeLeaveScope(id, "Pose6D");
    }

    template<class T>
    void readImpl(rw::math::Pose6D<T>& tmp, rw::common::InputArchive& iar, const std::string& id){
        rw::math::Vector3D<T> pos;
        rw::math::EAA<T> eaa;
        iar.readEnterScope(id, "Pose6D");
        iar.read( pos , "pos" );
        iar.read( eaa , "eaa" );
        iar.readLeaveScope(id, "Pose6D");
        tmp = rw::math::Pose6D<T>(pos,eaa);
    }

    // some explicit template specifications
    template<> void write(const rw::math::Pose6D<double>& tmp, rw::common::OutputArchive& oar, const std::string& id){writeImpl(tmp,oar,id);}
    template<> void write(const rw::math::Pose6D<float>& tmp, rw::common::OutputArchive& oar, const std::string& id){writeImpl(tmp,oar,id);}
    template<> void read(rw::math::Pose6D<double>& tmp, rw::common::InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);}
    template<> void read(rw::math::Pose6D<float>& tmp, rw::common::InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);}

}}} // end namespaces
