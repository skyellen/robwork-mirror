/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "FKTable.hpp"

#include "Frame.hpp"

using namespace rw::kinematics;
using namespace rw::math;

FKTable::FKTable(const State& state) :
    _state(state),
    _end(_transforms.end())
{}

const Transform3D<>& FKTable::get(const Frame& frame) const
{
    TransformMap::iterator p = _transforms.find(&frame);
    if (p == _end) {
        Entry entry(&frame);

        const Frame* parent = frame.getParent(_state);
        if (!parent)
            entry.transform = frame.getTransform(_state);
        else 
            frame.getTransform(get(*parent), _state, entry.transform);

        return _transforms.insert(entry).first->transform;
    } else
        return p->transform;

    /*

    Version based on std::map<>:

    TransformMap::iterator p = _transforms.find(&frame);

    if (p == _transforms.end()) {
        const Frame* parent = frame.getParent(_state);
        if (!parent) {
            const Transform3D<>& local = frame.getTransform(_state);
            p = _transforms.insert(Entry(&frame, local)).first;
        } else {
            p = _transforms.insert(Entry(&frame, Transform3D<>())).first;
            frame.getTransform(get(*parent), _state, p->second);
        }
    }
    return p->second;
    */
}
