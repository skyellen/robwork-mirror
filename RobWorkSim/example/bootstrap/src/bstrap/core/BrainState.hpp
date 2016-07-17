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

#ifndef BRAINSTATE_HPP_
#define BRAINSTATE_HPP_

#include <rw/common/PropertyMap.hpp>
#include <rw/kinematics/State.hpp>

class BrainState {
public:
    rw::common::PropertyMap _pmap;
    rw::kinematics::State _rwstate;// yes, this is cheating a bit... but it makes life easier

    BrainState(rw::kinematics::State& state):_rwstate(state){}

    void save(const std::string& filename);

    static BrainState load(const std::string& filename);

    bool equal(const BrainState& state){
        //TODO: implement comparison
        return true;
    }

    rw::common::PropertyMap& getMap(){ return _pmap; }

    const rw::common::PropertyMap& getMap() const { return _pmap; }

    rw::kinematics::State& getRobWorkState(){ return _rwstate; }
    void setRobWorkState(rw::kinematics::State& state){ _rwstate = state; }
};

#endif
