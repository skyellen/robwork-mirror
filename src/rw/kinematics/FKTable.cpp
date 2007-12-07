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

using namespace rw::kinematics;
using namespace rw::math;

FKTable::FKTable(const State& state) :
    _state(state)
{}

Transform3D<> FKTable::get(const Frame& frame) const
{
    TransformMap::iterator p = _transforms.find(&frame);

    if (p != _transforms.end())
        return p->second;
    else {
        const Frame* parent = frame.getParent(_state);
        const Transform3D<>& local = frame.getTransform(_state);

        if (!parent) {
            const Transform3D<>& result = local;
            _transforms.insert(std::make_pair(&frame, result));
            return result;
        } else {
            const Transform3D<>& result = get(*parent) * local;
            _transforms.insert(std::make_pair(&frame, result));
            return result;
        }
    }
}
