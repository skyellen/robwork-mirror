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


#include "FKTable.hpp"

#include "Frame.hpp"

using namespace rw::kinematics;
using namespace rw::math;

FKTable::FKTable(const State* state) :
    _sp(state),
    _transforms(150)
{}

FKTable::FKTable(const State& state) :
    _state(state),
    _transforms(Transform3D<>::identity(), 150)
{
    _sp = &_state;
}

void FKTable::reset(const State& state){
    _state = state;
    _sp = &_state;
    _transforms.clear();
}

const Transform3D<>& FKTable::get(const Frame& frame) const
{
    /*
      Version based on kinematics::FrameMap:
    */
    const bool has = _transforms.has(frame);
    Transform3D<>& result = _transforms[frame];
    if (!has) {
        const Frame* parent = frame.getParent(getState());
        if (!parent)
            result = frame.getTransform(getState());
        else
            frame.multiplyTransform(get(*parent), getState(), result);
    }
    return result;

    /*
    Version based on boost::multi_index_container:

    TransformMap::iterator p = _transforms.find(&frame);
    if (p == _end) {
        Entry entry(&frame);

        const Frame* parent = frame.getParent(getState());
        if (!parent)
            entry.transform = frame.getTransform(getState());
        else
            frame.getTransform(get(*parent), getState(), entry.transform);

        return _transforms.insert(entry).first->transform;
    } else
        return p->transform;

    */

    /*
    Version based on std::map<>:

    TransformMap::iterator p = _transforms.find(&frame);

    if (p == _transforms.end()) {
        const Frame* parent = frame.getParent(getState());
        if (!parent) {
            const Transform3D<>& local = frame.getTransform(getState());
            p = _transforms.insert(Entry(&frame, local)).first;
        } else {
            p = _transforms.insert(Entry(&frame, Transform3D<>())).first;
            frame.getTransform(get(*parent), getState(), p->second);
        }
    }
    return p->second;
    */
}
