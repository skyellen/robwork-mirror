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

#include "Kinematics.hpp"

#include "FKRange.hpp"

using namespace rw::math;
using namespace rw::kinematics;

Transform3D<> Kinematics::WorldTframe(const Frame* to, const State& state)
{
    const Frame* f = to;

    Transform3D<> transform = Transform3D<>::Identity();
    while (f) {
        transform = f->getTransform(state) * transform;
        f = f->getParent(state);
    }

    return transform;
}

Transform3D<> Kinematics::FrameTframe(
    const Frame* from,
    const Frame* to,
    const State& state)
{
    RW_ASSERT(from != NULL);
    RW_ASSERT(to != NULL);
    
    FKRange range(from, to, state);
    return range.get(state);
}

namespace
{
    void findAllFramesHelper(
        Frame& frame,
        const State& state,
        std::vector<Frame*>& result)
    {
        result.push_back(&frame);

        Frame::iterator_pair children = frame.getChildren(state);
        for (Frame::iterator it = children.first; it != children.second; ++it) {
            findAllFramesHelper(*it, state, result);
        }
    }
}

std::vector<Frame*> Kinematics::FindAllFrames(
    Frame* root, const State& state)
{
    RW_ASSERT(root);

    std::vector<Frame*> result;
    findAllFramesHelper(*root, state, result);
    return result;
}

Kinematics::FrameMap Kinematics::BuildFrameMap(
    Frame& root, const State& state)
{
    FrameMap result;

    const std::vector<Frame*>& frames = Kinematics::FindAllFrames(&root, state);
    typedef std::vector<Frame*>::const_iterator I;
    for (I p = frames.begin(); p != frames.end(); ++p) {
        result.insert(std::make_pair((**p).getName(), *p));
    }

    return result;
}
