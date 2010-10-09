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


#include "Convert.hpp"

#include <rw/common/macros.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rws;
Convert::Convert(WorkCell* workcell)
{

	RW_ASSERT(workcell);
    setupDevices(workcell);
    setupFrames(
        workcell->getWorldFrame(),
        workcell->getDefaultState());
}

void Convert::setupDevices(WorkCell* workcell)
{
    const std::vector<Device*>& devices = workcell->getDevices();

    typedef std::vector<Device*>::const_iterator I;
    for (I it = devices.begin(); it != devices.end(); ++it) {
        if (dynamic_cast<SerialDevice*>(*it))
            _devicemap[*it] = SERIALDEVICE;
        //else if (dynamic_cast<Cartesian6DOFDevice*>(*it))
        //    _devicemap[*it] = CARTESIAN6DOFDEVICE;
        else if (dynamic_cast<TreeDevice*>(*it))
            _devicemap[*it] = TREEDEVICE;
        else if (dynamic_cast<ParallelDevice*>(*it))
            _devicemap[*it] = PARALLELDEVICE;
        else if (dynamic_cast<MobileDevice*>(*it))
            _devicemap[*it] = MOBILEDEVICE;
    }
}

void Convert::setupFrames(const Frame* frame, const State& state)
{
    if (dynamic_cast<const Joint*>(frame))
        _framemap[frame] = JOINT;
    /*else if (dynamic_cast<const DynamicObject*>(frame))
        _framemap[frame] = DYNAMICOBJECT;
    */
    Frame::const_iterator_pair children = frame->getChildren();
    for (Frame::const_iterator it = children.first; it != children.second; ++it) {
	setupFrames(&(*it), state);
    }

    Frame::const_iterator_pair dafChildren = frame->getDafChildren(state);
    for (Frame::const_iterator it = dafChildren.first; it != dafChildren.second; ++it) {
	setupFrames(&(*it), state);
    }
}
