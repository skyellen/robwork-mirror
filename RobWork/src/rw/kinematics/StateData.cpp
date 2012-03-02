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


#include "StateData.hpp"

#include "State.hpp"
#include <rw/common/macros.hpp>

using namespace rw::kinematics;

StateData::StateData(int size, const std::string& name):
    _id(-1),
    _size(size),
    _name(name),
    _hasCache(false)
{
    RW_ASSERT(0 <= size);
}

StateData::StateData(int size, const std::string& name, rw::common::Ptr<StateCache> cache):
    _id(-1),
    _size(size),
    _name(name),
    _hasCache(true),
    _cache(cache)
{
    RW_ASSERT(0 <= size);
}

rw::common::Ptr<StateCache> StateData::getCache(const State& state) const{
    if( _hasCache==false )
        return NULL; // stop early if we know size is 0
    return state.getCache(_id);
}

StateCache::Ptr StateData::getCache(State& state){
    if( _hasCache==false )
        return NULL; // stop early if we know size is 0
    return state.getCache(_id);
}

void StateData::setCache(rw::common::Ptr<StateCache> cache, State& state){
    if( _hasCache==false ) return; // stop early if we know size is 0
    state.setCache(_id, cache);
}
